#include "BMA400.h"
#include <string.h>

/* ----------------- Registers ----------------- */
#define BMA400_REG_CHIP_ID           0x00
#define BMA400_CHIP_ID_VAL           0x90

#define BMA400_REG_ACC_DATA          0x04  /* 0x04..0x09: X/Y/Z (LSB/MSB) */
#define BMA400_REG_INT_STAT0         0x0E

#define BMA400_REG_ACC_CONFIG0       0x19
#define BMA400_REG_ACC_CONFIG1       0x1A

#define BMA400_REG_INT_CONFIG0       0x1F
#define BMA400_REG_INT_CONFIG1       0x20
#define BMA400_REG_INT1_MAP          0x21
#define BMA400_REG_INT12_IO_CTRL     0x24

#define BMA400_REG_ORIENTCH_CONFIG0  0x35
#define BMA400_REG_ORIENTCH_CONFIG1  0x36
#define BMA400_REG_ORIENTCH_CONFIG3  0x38

/* ----------------- Bit helpers ----------------- */
static inline uint8_t BITU(uint8_t n) { return (uint8_t)(1u << n); }

/* INT_CONFIG0 (0x1F) */
#define BMA400_INT_CONFIG0_ORIENTCH_EN   BITU(1)

/* INT_CONFIG1 (0x20) */
#define BMA400_INT_CONFIG1_LATCH_INT     BITU(7)   /* 1=latch, 0=non-latched */

/* INT1_MAP (0x21) */
#define BMA400_INT1_MAP_ORIENTCH         BITU(1)

/* INT_STAT0 (0x0E) */
#define BMA400_INT_STAT0_ORIENTCH        BITU(1)

/* INT12_IO_CTRL (0x24) */
#define BMA400_INT12_IO_INT1_LVL         BITU(1)   /* 1=active high, 0=active low */
#define BMA400_INT12_IO_INT1_OD          BITU(2)   /* 1=open drain, 0=push-pull */

/* ORIENTCH_CONFIG0 (0x35) */
#define BMA400_ORIENT_Z_EN               BITU(7)
#define BMA400_ORIENT_Y_EN               BITU(6)
#define BMA400_ORIENT_X_EN               BITU(5)
#define BMA400_ORIENT_DATA_SRC_LP        BITU(4)   /* 1=acc_filt_lp, 0=acc_filt2 */

#define BMA400_ORIENT_REFU_ONETIME_FILT2 (1u << 2)
#define BMA400_ORIENT_REFU_ONETIME_LP    (2u << 2)

/* ACC_CONFIG0 (0x19) */
#define BMA400_PWRMODE_NORMAL            (0x02u)   /* bits[1:0] */

/* ACC_CONFIG1 (0x1A) helpers:
 * odr=0x08 => 100Hz
 * range=0x00 => +/-2g
 * osr=0x00
 */
static uint8_t make_acc_config1(uint8_t range, uint8_t osr, uint8_t odr)
{
    return (uint8_t)((range << 6) | (osr << 4) | (odr & 0x0F));
}

/* ----------------- I2C helpers ----------------- */
static uint16_t dev_addr_8bit(const BMA400_Orient_t *dev)
{
    return (uint16_t)(dev->i2c_addr_7bit << 1);
}

static HAL_StatusTypeDef bma400_read(const BMA400_Orient_t *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(dev->hi2c, dev_addr_8bit(dev), reg,
                            I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

static HAL_StatusTypeDef bma400_write(const BMA400_Orient_t *dev, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Write(dev->hi2c, dev_addr_8bit(dev), reg,
                             I2C_MEMADD_SIZE_8BIT, (uint8_t*)buf, len, 100);
}

static HAL_StatusTypeDef bma400_write_u8(const BMA400_Orient_t *dev, uint8_t reg, uint8_t val)
{
    return bma400_write(dev, reg, &val, 1);
}

static HAL_StatusTypeDef bma400_update_bits(const BMA400_Orient_t *dev, uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t tmp = 0;
    HAL_StatusTypeDef st = bma400_read(dev, reg, &tmp, 1);
    if (st != HAL_OK) return st;
    tmp = (uint8_t)((tmp & ~mask) | (value & mask));
    return bma400_write_u8(dev, reg, tmp);
}

/* ----------------- Raw unpack (12-bit two's complement) ----------------- */
static int16_t unpack_12bit(uint8_t lsb, uint8_t msb)
{
    /* upper nibble of MSB contains bits[11:8] */
    int16_t raw = (int16_t)((((uint16_t)(msb & 0x0F)) << 8) | (uint16_t)lsb);
    if (raw & 0x0800) raw |= (int16_t)0xF000; /* sign-extend */
    return raw;
}

/* ----------------- Public API ----------------- */
void BMA400_Orient_Init(BMA400_Orient_t *dev, I2C_HandleTypeDef *hi2c, uint8_t i2c_addr_7bit)
{
    if (!dev) return;
    dev->hi2c = hi2c;
    dev->i2c_addr_7bit = i2c_addr_7bit;
}

HAL_StatusTypeDef BMA400_Orient_EnableInt1(BMA400_Orient_t *dev, const BMA400_OrientCfg_t *cfg)
{
    if (!dev || !dev->hi2c || !cfg) return HAL_ERROR;

    /* Verify chip ID */
    uint8_t chip = 0;
    HAL_StatusTypeDef st = bma400_read(dev, BMA400_REG_CHIP_ID, &chip, 1);
    if (st != HAL_OK) return st;
    if (chip != BMA400_CHIP_ID_VAL) return HAL_ERROR;

    /* Accel normal mode */
    st = bma400_update_bits(dev, BMA400_REG_ACC_CONFIG0, 0x03u, BMA400_PWRMODE_NORMAL);
    if (st != HAL_OK) return st;

    /* 100 Hz, +/-2g */
    const uint8_t acc1 = make_acc_config1(/*range*/0x00, /*osr*/0x00, /*odr*/0x08);
    st = bma400_write_u8(dev, BMA400_REG_ACC_CONFIG1, acc1);
    if (st != HAL_OK) return st;

    /* INT1 electrical: level + push-pull/open-drain */
    uint8_t io_bits = 0;
    if (cfg->int1_active_high) io_bits |= BMA400_INT12_IO_INT1_LVL;
    if (!cfg->int1_push_pull)  io_bits |= BMA400_INT12_IO_INT1_OD; /* open drain */

    st = bma400_update_bits(dev,
                            BMA400_REG_INT12_IO_CTRL,
                            (uint8_t)(BMA400_INT12_IO_INT1_LVL | BMA400_INT12_IO_INT1_OD),
                            io_bits);
    if (st != HAL_OK) return st;

    /* Latch configuration */
    if (cfg->non_latched) {
        st = bma400_update_bits(dev, BMA400_REG_INT_CONFIG1, BMA400_INT_CONFIG1_LATCH_INT, 0u);
    } else {
        st = bma400_update_bits(dev, BMA400_REG_INT_CONFIG1, BMA400_INT_CONFIG1_LATCH_INT, BMA400_INT_CONFIG1_LATCH_INT);
    }
    if (st != HAL_OK) return st;

    /* Orientation-change config0: enable only chosen axes + choose data source + one-time ref update */
    uint8_t orient0 = 0;

    /* Axis selection */
    if (cfg->axis_mask & BMA400_AXIS_X) orient0 |= BMA400_ORIENT_X_EN;
    if (cfg->axis_mask & BMA400_AXIS_Y) orient0 |= BMA400_ORIENT_Y_EN;
    if (cfg->axis_mask & BMA400_AXIS_Z) orient0 |= BMA400_ORIENT_Z_EN;

    /* If none selected, fail fast */
    if ((orient0 & (BMA400_ORIENT_X_EN | BMA400_ORIENT_Y_EN | BMA400_ORIENT_Z_EN)) == 0u) {
        return HAL_ERROR;
    }

    if (cfg->use_lowpass_1hz) {
        orient0 |= (uint8_t)(BMA400_ORIENT_DATA_SRC_LP | BMA400_ORIENT_REFU_ONETIME_LP);
    } else {
        /* data_src = filt2 (bit4=0) */
        orient0 |= (uint8_t)(BMA400_ORIENT_REFU_ONETIME_FILT2);
    }

    st = bma400_write_u8(dev, BMA400_REG_ORIENTCH_CONFIG0, orient0);
    if (st != HAL_OK) return st;

    /* Threshold + duration */
    st = bma400_write_u8(dev, BMA400_REG_ORIENTCH_CONFIG1, cfg->threshold_lsb);
    if (st != HAL_OK) return st;

    st = bma400_write_u8(dev, BMA400_REG_ORIENTCH_CONFIG3, cfg->duration_lsb);
    if (st != HAL_OK) return st;

    /* Enable orientation-change interrupt generation */
    st = bma400_update_bits(dev,
                            BMA400_REG_INT_CONFIG0,
                            BMA400_INT_CONFIG0_ORIENTCH_EN,
                            BMA400_INT_CONFIG0_ORIENTCH_EN);
    if (st != HAL_OK) return st;

    /* Map to INT1 */
    st = bma400_update_bits(dev,
                            BMA400_REG_INT1_MAP,
                            BMA400_INT1_MAP_ORIENTCH,
                            BMA400_INT1_MAP_ORIENTCH);
    if (st != HAL_OK) return st;

    /* Clear any stale status by reading it once */
    uint8_t stat0 = 0;
    (void)bma400_read(dev, BMA400_REG_INT_STAT0, &stat0, 1);

    return HAL_OK;
}

HAL_StatusTypeDef BMA400_Orient_ReadIntStatus(BMA400_Orient_t *dev, bool *out_orientch)
{
    if (!dev || !dev->hi2c || !out_orientch) return HAL_ERROR;

    uint8_t stat0 = 0;
    HAL_StatusTypeDef st = bma400_read(dev, BMA400_REG_INT_STAT0, &stat0, 1);
    if (st != HAL_OK) return st;

    *out_orientch = ((stat0 & BMA400_INT_STAT0_ORIENTCH) != 0);
    return HAL_OK;
}

HAL_StatusTypeDef BMA400_Orient_RearmReference(BMA400_Orient_t *dev, bool use_lowpass_1hz)
{
    if (!dev || !dev->hi2c) return HAL_ERROR;

    uint8_t orient0 = 0;
    HAL_StatusTypeDef st = bma400_read(dev, BMA400_REG_ORIENTCH_CONFIG0, &orient0, 1);
    if (st != HAL_OK) return st;

    /* Clear data_src (bit4) and refu bits [3:2] */
    orient0 &= (uint8_t)~(BITU(4) | BITU(3) | BITU(2));

    if (use_lowpass_1hz) {
        orient0 |= (uint8_t)(BMA400_ORIENT_DATA_SRC_LP | BMA400_ORIENT_REFU_ONETIME_LP);
    } else {
        orient0 |= (uint8_t)(BMA400_ORIENT_REFU_ONETIME_FILT2);
    }

    return bma400_write_u8(dev, BMA400_REG_ORIENTCH_CONFIG0, orient0);
}

HAL_StatusTypeDef BMA400_ReadXYZ_Raw12(BMA400_Orient_t *dev, int16_t *x, int16_t *y, int16_t *z)
{
    if (!dev || !dev->hi2c || !x || !y || !z) return HAL_ERROR;

    uint8_t b[6] = {0};
    HAL_StatusTypeDef st = bma400_read(dev, BMA400_REG_ACC_DATA, b, 6);
    if (st != HAL_OK) return st;

    *x = unpack_12bit(b[0], b[1]);
    *y = unpack_12bit(b[2], b[3]);
    *z = unpack_12bit(b[4], b[5]);
    return HAL_OK;
}

bool BMA400_IsFaceToMe(int16_t x, int16_t y, int16_t z,
                       BMA400_FaceAxis_t face_axis,
                       int16_t threshold_raw)
{
    /* threshold_raw should be positive, e.g. 768 for ~0.75g if ~1024 LSB/g */
    if (threshold_raw < 0) threshold_raw = (int16_t)-threshold_raw;

    switch (face_axis)
    {
        case BMA400_FACE_AXIS_X_POS: return (x >  threshold_raw);
        case BMA400_FACE_AXIS_X_NEG: return (x < -threshold_raw);
        case BMA400_FACE_AXIS_Y_POS: return (y >  threshold_raw);
        case BMA400_FACE_AXIS_Y_NEG: return (y < -threshold_raw);
        case BMA400_FACE_AXIS_Z_POS: return (z >  threshold_raw);
        case BMA400_FACE_AXIS_Z_NEG: return (z < -threshold_raw); //Bylo tam -threshold_raw
        default: return false;
    }
}
