#include <furi_hal_i2c.h>
#include <furi_hal_version.h>
#include <furi_hal_power.h>
#include <furi_hal_cortex.h>
#include <furi_hal_interrupt.h>

#include <stm32wbxx_ll_dma.h>
#include <stm32wbxx_ll_i2c.h>
#include <stm32wbxx_ll_gpio.h>
#include <stm32wbxx_ll_cortex.h>
#include <furi.h>

#define TAG "FuriHalI2C"

#define I2C_DMA DMA2
#define I2C_DMA_RX_CHANNEL LL_DMA_CHANNEL_3
#define I2C_DMA_TX_CHANNEL LL_DMA_CHANNEL_4
#define I2C_DMA_RX_IRQ FuriHalInterruptIdDma2Ch3
#define I2C_DMA_TX_IRQ FuriHalInterruptIdDma2Ch4
#define I2C_DMA_RX_DEF I2C_DMA, I2C_DMA_RX_CHANNEL
#define I2C_DMA_TX_DEF I2C_DMA, I2C_DMA_TX_CHANNEL

static FuriSemaphore* i2c_dma_lock = NULL;
static FuriSemaphore* i2c_dma_completed = NULL;

void furi_hal_i2c_init_early() {
    furi_hal_i2c_bus_power.callback(&furi_hal_i2c_bus_power, FuriHalI2cBusEventInit);
}

void furi_hal_i2c_deinit_early() {
    furi_hal_i2c_bus_power.callback(&furi_hal_i2c_bus_power, FuriHalI2cBusEventDeinit);
}

void furi_hal_i2c_init() {
    i2c_dma_lock = furi_semaphore_alloc(1, 1);
    i2c_dma_completed = furi_semaphore_alloc(1, 1);

    furi_hal_i2c_bus_external.callback(&furi_hal_i2c_bus_external, FuriHalI2cBusEventInit);
    FURI_LOG_I(TAG, "Init OK");
}

void furi_hal_i2c_acquire(FuriHalI2cBusHandle* handle) {
    furi_hal_power_insomnia_enter();
    // Lock bus access
    handle->bus->callback(handle->bus, FuriHalI2cBusEventLock);
    // Ensuree that no active handle set
    furi_check(handle->bus->current_handle == NULL);
    // Set current handle
    handle->bus->current_handle = handle;
    // Activate bus
    handle->bus->callback(handle->bus, FuriHalI2cBusEventActivate);
    // Activate handle
    handle->callback(handle, FuriHalI2cBusHandleEventActivate);
}

void furi_hal_i2c_release(FuriHalI2cBusHandle* handle) {
    // Ensure that current handle is our handle
    furi_check(handle->bus->current_handle == handle);
    // Deactivate handle
    handle->callback(handle, FuriHalI2cBusHandleEventDeactivate);
    // Deactivate bus
    handle->bus->callback(handle->bus, FuriHalI2cBusEventDeactivate);
    // Reset current handle
    handle->bus->current_handle = NULL;
    // Unlock bus
    handle->bus->callback(handle->bus, FuriHalI2cBusEventUnlock);
    furi_hal_power_insomnia_exit();
}

bool furi_hal_i2c_tx(
    FuriHalI2cBusHandle* handle,
    uint8_t address,
    const uint8_t* data,
    uint8_t size,
    uint32_t timeout) {
    furi_check(handle->bus->current_handle == handle);
    furi_assert(timeout > 0);

    bool ret = true;
    FuriHalCortexTimer timer = furi_hal_cortex_timer_get(timeout * 1000);

    do {
        while(LL_I2C_IsActiveFlag_BUSY(handle->bus->i2c)) {
            if(furi_hal_cortex_timer_is_expired(timer)) {
                ret = false;
                break;
            }
        }

        if(!ret) {
            break;
        }

        LL_I2C_HandleTransfer(
            handle->bus->i2c,
            address,
            LL_I2C_ADDRSLAVE_7BIT,
            size,
            LL_I2C_MODE_AUTOEND,
            LL_I2C_GENERATE_START_WRITE);

        while(!LL_I2C_IsActiveFlag_STOP(handle->bus->i2c) || size > 0) {
            if(LL_I2C_IsActiveFlag_TXIS(handle->bus->i2c)) {
                LL_I2C_TransmitData8(handle->bus->i2c, (*data));
                data++;
                size--;
            }

            if(furi_hal_cortex_timer_is_expired(timer)) {
                ret = false;
                break;
            }
        }

        LL_I2C_ClearFlag_STOP(handle->bus->i2c);
    } while(0);

    return ret;
}

bool furi_hal_i2c_rx(
    FuriHalI2cBusHandle* handle,
    uint8_t address,
    uint8_t* data,
    uint8_t size,
    uint32_t timeout) {
    furi_check(handle->bus->current_handle == handle);
    furi_assert(timeout > 0);

    bool ret = true;
    FuriHalCortexTimer timer = furi_hal_cortex_timer_get(timeout * 1000);

    do {
        while(LL_I2C_IsActiveFlag_BUSY(handle->bus->i2c)) {
            if(furi_hal_cortex_timer_is_expired(timer)) {
                ret = false;
                break;
            }
        }

        if(!ret) {
            break;
        }

        LL_I2C_HandleTransfer(
            handle->bus->i2c,
            address,
            LL_I2C_ADDRSLAVE_7BIT,
            size,
            LL_I2C_MODE_AUTOEND,
            LL_I2C_GENERATE_START_READ);

        while(!LL_I2C_IsActiveFlag_STOP(handle->bus->i2c) || size > 0) {
            if(LL_I2C_IsActiveFlag_RXNE(handle->bus->i2c)) {
                *data = LL_I2C_ReceiveData8(handle->bus->i2c);
                data++;
                size--;
            }

            if(furi_hal_cortex_timer_is_expired(timer)) {
                ret = false;
                break;
            }
        }

        LL_I2C_ClearFlag_STOP(handle->bus->i2c);
    } while(0);

    return ret;
}

bool furi_hal_i2c_trx(
    FuriHalI2cBusHandle* handle,
    uint8_t address,
    const uint8_t* tx_data,
    uint8_t tx_size,
    uint8_t* rx_data,
    uint8_t rx_size,
    uint32_t timeout) {
    if(furi_hal_i2c_tx(handle, address, tx_data, tx_size, timeout) &&
       furi_hal_i2c_rx(handle, address, rx_data, rx_size, timeout)) {
        return true;
    } else {
        return false;
    }
}

static void i2c_dma_isr() {
    FURI_LOG_E(TAG, "I2C DMA ISR\r\n");
#if I2C_DMA_RX_CHANNEL == LL_DMA_CHANNEL_3
    if(LL_DMA_IsActiveFlag_TC3(I2C_DMA) && LL_DMA_IsEnabledIT_TC(I2C_DMA_RX_DEF)) {
        LL_DMA_ClearFlag_TC3(I2C_DMA);
        furi_check(furi_semaphore_release(i2c_dma_completed) == FuriStatusOk);
    }
#else
#error Update this code. Would you kindly?
#endif

#if I2C_DMA_TX_CHANNEL == LL_DMA_CHANNEL_4
    if(LL_DMA_IsActiveFlag_TC4(I2C_DMA) && LL_DMA_IsEnabledIT_TC(I2C_DMA_TX_DEF)) {
        LL_DMA_ClearFlag_TC4(I2C_DMA);
        furi_check(furi_semaphore_release(i2c_dma_completed) == FuriStatusOk);
    }
#else
#error Update this code. Would you kindly?
#endif
}

bool furi_hal_i2c_bus_trx_dma(
    FuriHalI2cBusHandle* handle,
    uint8_t address,
    const uint8_t* tx_data,
    uint8_t tx_size,
    uint8_t* rx_data,
    uint8_t rx_size,
    uint32_t timeout_ms) {
    furi_assert(handle);
    furi_assert(handle->bus->current_handle == handle);
    furi_assert(tx_size > 0);
    furi_assert(rx_size > 0);
    furi_check(furi_semaphore_acquire(i2c_dma_lock, FuriWaitForever) == FuriStatusOk);

    const uint32_t dma_dummy_u32 = 0xFFFFFFFF;

    bool ret = true;
    I2C_TypeDef* i2c = handle->bus->i2c;
    uint32_t dma_rx_req;
    uint32_t dma_tx_req;

    if(i2c == I2C1) {
        dma_rx_req = LL_DMAMUX_REQ_I2C1_RX;
        dma_tx_req = LL_DMAMUX_REQ_I2C1_TX;
    } else if(i2c == I2C3) {
        dma_rx_req = LL_DMAMUX_REQ_I2C3_RX;
        dma_tx_req = LL_DMAMUX_REQ_I2C3_TX;
    } else {
        furi_crash(NULL);
    }

    if(rx_data == NULL) {
        LL_DMA_InitTypeDef dma_config = {0};
        dma_config.PeriphOrM2MSrcAddress = (uint32_t) & (i2c->TXDR);
        dma_config.MemoryOrM2MDstAddress = (uint32_t)tx_data;
        dma_config.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
        dma_config.Mode = LL_DMA_MODE_NORMAL;
        dma_config.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
        dma_config.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
        dma_config.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
        dma_config.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
        dma_config.NbData = tx_size;
        dma_config.PeriphRequest = dma_tx_req;
        dma_config.Priority = LL_DMA_PRIORITY_MEDIUM;
        LL_DMA_Init(I2C_DMA_TX_DEF, &dma_config);

#if I2C_DMA_TX_CHANNEL == LL_DMA_CHANNEL_4
        LL_DMA_ClearFlag_TC4(I2C_DMA);
#else
#error Update this code. Would you kindly?
#endif

        furi_hal_interrupt_set_isr(I2C_DMA_TX_IRQ, i2c_dma_isr, NULL);

        bool dma_tx_was_enabled = LL_I2C_IsEnabledDMAReq_TX(i2c);
        if(!dma_tx_was_enabled) {
            LL_I2C_EnableDMAReq_TX(i2c);
        }

        // acquire semaphore before enabling DMA
        furi_check(furi_semaphore_acquire(i2c_dma_completed, timeout_ms) == FuriStatusOk);

        LL_DMA_EnableIT_TC(I2C_DMA_TX_DEF);
        LL_DMA_EnableChannel(I2C_DMA_TX_DEF);

        // and wait for it to be released (DMA transfer complete)
        if(furi_semaphore_acquire(i2c_dma_completed, timeout_ms) != FuriStatusOk) {
            ret = false;
            FURI_LOG_E(TAG, "DMA timeout 1\r\n");
        }
        // release semaphore, because we are using it as a flag
        furi_semaphore_release(i2c_dma_completed);

        LL_DMA_DisableIT_TC(I2C_DMA_TX_DEF);
        LL_DMA_DisableChannel(I2C_DMA_TX_DEF);
        if(!dma_tx_was_enabled) {
            LL_I2C_DisableDMAReq_TX(i2c);
        }
        furi_hal_interrupt_set_isr(I2C_DMA_TX_IRQ, NULL, NULL);

        LL_DMA_DeInit(I2C_DMA_TX_DEF);
    } else {
        uint32_t tx_mem_increase_mode;

        if(tx_data == NULL) {
            tx_data = (const uint8_t*)&dma_dummy_u32;
            tx_mem_increase_mode = LL_DMA_PERIPH_NOINCREMENT;
        } else {
            tx_mem_increase_mode = LL_DMA_MEMORY_INCREMENT;
        }

        LL_DMA_InitTypeDef dma_config = {0};
        dma_config.PeriphOrM2MSrcAddress = (uint32_t) & (i2c->TXDR);
        dma_config.MemoryOrM2MDstAddress = (uint32_t)tx_size;
        dma_config.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
        dma_config.Mode = LL_DMA_MODE_NORMAL;
        dma_config.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
        dma_config.MemoryOrM2MDstIncMode = tx_mem_increase_mode;
        dma_config.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
        dma_config.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
        dma_config.NbData = tx_size;
        dma_config.PeriphRequest = dma_tx_req;
        dma_config.Priority = LL_DMA_PRIORITY_MEDIUM;
        LL_DMA_Init(I2C_DMA_TX_DEF, &dma_config);

        dma_config.PeriphOrM2MSrcAddress = (uint32_t) & (i2c->RXDR);
        dma_config.MemoryOrM2MDstAddress = (uint32_t)rx_data;
        dma_config.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
        dma_config.Mode = LL_DMA_MODE_NORMAL;
        dma_config.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
        dma_config.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
        dma_config.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
        dma_config.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
        dma_config.NbData = rx_size;
        dma_config.PeriphRequest = dma_rx_req;
        dma_config.Priority = LL_DMA_PRIORITY_MEDIUM;
        LL_DMA_Init(I2C_DMA_RX_DEF, &dma_config);

#if I2C_DMA_RX_CHANNEL == LL_DMA_CHANNEL_3
        LL_DMA_ClearFlag_TC3(I2C_DMA);
#else
#error Update this code. Would you kindly?
#endif

        furi_hal_interrupt_set_isr(I2C_DMA_RX_IRQ, i2c_dma_isr, NULL);

        bool dma_tx_was_enabled = LL_I2C_IsEnabledDMAReq_TX(i2c);
        bool dma_rx_was_enabled = LL_I2C_IsEnabledDMAReq_RX(i2c);

        if(!dma_tx_was_enabled) {
            LL_I2C_EnableDMAReq_TX(i2c);
        }

        if(!dma_rx_was_enabled) {
            LL_I2C_EnableDMAReq_RX(i2c);
        }

        // acquire semaphore before enabling DMA
        furi_check(furi_semaphore_acquire(i2c_dma_completed, timeout_ms) == FuriStatusOk);

        LL_DMA_EnableIT_TC(I2C_DMA_RX_DEF);
        LL_DMA_EnableChannel(I2C_DMA_RX_DEF);
        LL_DMA_EnableChannel(I2C_DMA_TX_DEF);

        // and wait for it to be released (DMA transfer complete)
        if(furi_semaphore_acquire(i2c_dma_completed, timeout_ms) != FuriStatusOk) {
            ret = false;
            FURI_LOG_E(TAG, "DMA timeout 2\r\n");
        }
        // release semaphore, because we are using it as a flag
        furi_semaphore_release(i2c_dma_completed);

        LL_DMA_DisableIT_TC(I2C_DMA_RX_DEF);

        LL_DMA_DisableChannel(I2C_DMA_TX_DEF);
        LL_DMA_DisableChannel(I2C_DMA_RX_DEF);

        if(!dma_tx_was_enabled) {
            LL_I2C_DisableDMAReq_TX(i2c);
        }

        if(!dma_rx_was_enabled) {
            LL_I2C_DisableDMAReq_RX(i2c);
        }

        furi_hal_interrupt_set_isr(I2C_DMA_RX_IRQ, NULL, NULL);

        LL_DMA_DeInit(I2C_DMA_TX_DEF);
        LL_DMA_DeInit(I2C_DMA_RX_DEF);
    }

    // TODO(heimskr)
    // furi_hal_spi_bus_end_txrx(handle, timeout_ms);

    furi_check(furi_semaphore_release(i2c_dma_lock) == FuriStatusOk);

    return ret;
}

bool furi_hal_i2c_is_device_ready(FuriHalI2cBusHandle* handle, uint8_t i2c_addr, uint32_t timeout) {
    furi_check(handle);

    furi_check(handle->bus->current_handle == handle);
    furi_assert(timeout > 0);

    bool ret = true;
    FuriHalCortexTimer timer = furi_hal_cortex_timer_get(timeout * 1000);

    do {
        while(LL_I2C_IsActiveFlag_BUSY(handle->bus->i2c)) {
            if(furi_hal_cortex_timer_is_expired(timer)) {
                return false;
            }
        }

        handle->bus->i2c->CR2 =
            ((((uint32_t)(i2c_addr) & (I2C_CR2_SADD)) | (I2C_CR2_START) | (I2C_CR2_AUTOEND)) &
             (~I2C_CR2_RD_WRN));

        while((!LL_I2C_IsActiveFlag_NACK(handle->bus->i2c)) &&
              (!LL_I2C_IsActiveFlag_STOP(handle->bus->i2c))) {
            if(furi_hal_cortex_timer_is_expired(timer)) {
                return false;
            }
        }

        if(LL_I2C_IsActiveFlag_NACK(handle->bus->i2c)) {
            while(!LL_I2C_IsActiveFlag_STOP(handle->bus->i2c)) {
                if(furi_hal_cortex_timer_is_expired(timer)) {
                    return false;
                }
            }

            LL_I2C_ClearFlag_NACK(handle->bus->i2c);

            // Clear STOP Flag generated by autoend
            LL_I2C_ClearFlag_STOP(handle->bus->i2c);

            // Generate actual STOP
            LL_I2C_GenerateStopCondition(handle->bus->i2c);

            ret = false;
        }

        while(!LL_I2C_IsActiveFlag_STOP(handle->bus->i2c)) {
            if(furi_hal_cortex_timer_is_expired(timer)) {
                return false;
            }
        }

        LL_I2C_ClearFlag_STOP(handle->bus->i2c);
    } while(0);

    return ret;
}

bool furi_hal_i2c_read_reg_8(
    FuriHalI2cBusHandle* handle,
    uint8_t i2c_addr,
    uint8_t reg_addr,
    uint8_t* data,
    uint32_t timeout) {
    furi_check(handle);

    return furi_hal_i2c_trx(handle, i2c_addr, &reg_addr, 1, data, 1, timeout);
}

bool furi_hal_i2c_read_reg_16(
    FuriHalI2cBusHandle* handle,
    uint8_t i2c_addr,
    uint8_t reg_addr,
    uint16_t* data,
    uint32_t timeout) {
    furi_check(handle);

    uint8_t reg_data[2];
    bool ret = furi_hal_i2c_trx(handle, i2c_addr, &reg_addr, 1, reg_data, 2, timeout);
    *data = (reg_data[0] << 8) | (reg_data[1]);

    return ret;
}

bool furi_hal_i2c_read_mem(
    FuriHalI2cBusHandle* handle,
    uint8_t i2c_addr,
    uint8_t mem_addr,
    uint8_t* data,
    uint8_t len,
    uint32_t timeout) {
    furi_check(handle);

    return furi_hal_i2c_trx(handle, i2c_addr, &mem_addr, 1, data, len, timeout);
}

bool furi_hal_i2c_write_reg_8(
    FuriHalI2cBusHandle* handle,
    uint8_t i2c_addr,
    uint8_t reg_addr,
    uint8_t data,
    uint32_t timeout) {
    furi_check(handle);

    uint8_t tx_data[2];
    tx_data[0] = reg_addr;
    tx_data[1] = data;

    return furi_hal_i2c_tx(handle, i2c_addr, (const uint8_t*)&tx_data, 2, timeout);
}

bool furi_hal_i2c_write_reg_16(
    FuriHalI2cBusHandle* handle,
    uint8_t i2c_addr,
    uint8_t reg_addr,
    uint16_t data,
    uint32_t timeout) {
    furi_check(handle);

    uint8_t tx_data[3];
    tx_data[0] = reg_addr;
    tx_data[1] = (data >> 8) & 0xFF;
    tx_data[2] = data & 0xFF;

    return furi_hal_i2c_tx(handle, i2c_addr, (const uint8_t*)&tx_data, 3, timeout);
}

bool furi_hal_i2c_write_mem(
    FuriHalI2cBusHandle* handle,
    uint8_t i2c_addr,
    uint8_t mem_addr,
    uint8_t* data,
    uint8_t len,
    uint32_t timeout) {
    furi_check(handle);

    furi_check(handle->bus->current_handle == handle);
    furi_assert(timeout > 0);

    bool ret = true;
    uint8_t size = len + 1;
    FuriHalCortexTimer timer = furi_hal_cortex_timer_get(timeout * 1000);

    do {
        while(LL_I2C_IsActiveFlag_BUSY(handle->bus->i2c)) {
            if(furi_hal_cortex_timer_is_expired(timer)) {
                ret = false;
                break;
            }
        }

        if(!ret) {
            break;
        }

        LL_I2C_HandleTransfer(
            handle->bus->i2c,
            i2c_addr,
            LL_I2C_ADDRSLAVE_7BIT,
            size,
            LL_I2C_MODE_AUTOEND,
            LL_I2C_GENERATE_START_WRITE);

        while(!LL_I2C_IsActiveFlag_STOP(handle->bus->i2c) || size > 0) {
            if(LL_I2C_IsActiveFlag_TXIS(handle->bus->i2c)) {
                if(size == len + 1) {
                    LL_I2C_TransmitData8(handle->bus->i2c, mem_addr);
                } else {
                    LL_I2C_TransmitData8(handle->bus->i2c, (*data));
                    data++;
                }
                size--;
            }

            if(furi_hal_cortex_timer_is_expired(timer)) {
                ret = false;
                break;
            }
        }

        LL_I2C_ClearFlag_STOP(handle->bus->i2c);
    } while(0);

    return ret;
}
