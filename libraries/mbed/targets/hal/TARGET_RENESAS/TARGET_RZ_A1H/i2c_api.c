/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "mbed_assert.h"
#include "i2c_api.h"
#include "cmsis.h"
#include "pinmap.h"
#include "r_typedefs.h"

#include "riic_iodefine.h"
#include "RZ_A1_Init.h"
#include "MBRZA1H.h"

#if defined(__ARMCC_VERSION)
#define FLOAT64_T   float64_t
#elif defined(__GNUC__)
#define FLOAT64_T   float
#endif

volatile struct st_riic *RIIC[] = RIIC_ADDRESS_LIST;

#define REG(N) \
    RIIC[obj->i2c]->RIICn##N

/* RIICnCR1 */
#define CR1_RST   (1 << 6)
#define CR1_ICE   (1 << 7)

/* RIICnCR2 */
#define CR2_ST    (1 << 1)
#define CR2_SP    (1 << 3)
#define CR2_NACKF (1 << 4)
#define CR2_BBSY  (1 << 7)

/* RIICnMR3 */
#define MR3_ACKBT (1 << 3)
#define MR3_ACKWP (1 << 4)
#define MR3_WAIT  (1 << 6)

/* RIICnSR2 */
#define SR2_STOP  (1 << 3)
#define SR2_NACKF (1 << 4)
#define SR2_RDRF  (1 << 5)
#define SR2_TEND  (1 << 6)
#define SR2_TDRE  (1 << 7)

#define TIMEOUT_1S    (3600000)  /* Loop counter : Time-out is about 1s. By 3600000 loops, measured value is 969ms. */

static const PinMap PinMap_I2C_SDA[] = {
    {P1_1 , I2C_0, 1},
    {P1_3 , I2C_1, 1},
    {P1_7 , I2C_3, 1},
    {NC   , NC   , 0}
};

static const PinMap PinMap_I2C_SCL[] = {
    {P1_0 , I2C_0, 1},
    {P1_2 , I2C_1, 1},
    {P1_6 , I2C_3, 1},
    {NC   , NC,    0}
};


/* Clear the Transmit data Empty TDRE */
static inline int i2c_addressed(i2c_t *obj) {
    volatile int sar0 = (REG(SR1.UINT8[0])&1),
                 trs  = (REG(CR2.UINT8[0])&0x20) >> 5;
    return sar0 | (trs <<1);
}

static inline int i2c_status(i2c_t *obj) {
    return REG(SR2.UINT8[0]);
}

static inline void i2c_clear_TDRE(i2c_t *obj) {
    REG(SR2.UINT32) &= ~SR2_TDRE;
}

static inline int i2c_wait_RDRF(i2c_t *obj) {
    int timeout = 0;
    
    /* There is no timeout, but the upper limit value is set to avoid an infinite loop. */
    while (!(i2c_status(obj) & SR2_RDRF)) {
        timeout ++;
        if (timeout >= TIMEOUT_1S) {
            return -1;
        }
    }

    return 0;
}

static void i2c_reg_reset(i2c_t *obj) {
    /* full reset */
    REG(CR1.UINT8[0]) &= ~CR1_ICE; // CR1.ICE off
    REG(CR1.UINT8[0]) |=  CR1_RST; // CR1.IICRST on
    REG(CR1.UINT8[0]) |=  CR1_ICE; // CR1.ICE on

    REG(MR1.UINT8[0])  =  0x08;    // P_phi /x  9bit (including Ack)
    REG(SER.UINT8[0])  =  0x00;    // no slave addr enabled

    /* set frequency */
    REG(MR1.UINT8[0]) |=  obj->pclk_bit;
    REG(BRL.UINT8[0])  =  obj->width_low;
    REG(BRH.UINT8[0])  =  obj->width_hi;

    REG(MR2.UINT8[0])  =  0x07;
    REG(MR3.UINT8[0])  =  0x00;

    REG(FER.UINT8[0])  =  0x72;    // SCLE, NFE enabled, TMOT
    REG(IER.UINT8[0])  =  0x00;    // no interrupt

    REG(CR1.UINT32) &= ~CR1_RST;   // CR1.IICRST negate reset
}

/* Wait until the Trans Data Empty (TDRE) is set */
static int i2c_wait_TDRE(i2c_t *obj) {
    int timeout = 0;

    /* There is no timeout, but the upper limit value is set to avoid an infinite loop. */
    while (!(i2c_status(obj) & SR2_TDRE)) {
        timeout ++;
        if (timeout >= TIMEOUT_1S) {
            return -1;
        }
    }

    return 0;
}

static int i2c_wait_TEND(i2c_t *obj) {
    int timeout = 0;
    
    /* There is no timeout, but the upper limit value is set to avoid an infinite loop. */
    while (!(i2c_status(obj) & SR2_TEND)) {
        timeout ++;
        if (timeout >= TIMEOUT_1S) {
            return -1;
        }
    }

    return 0;
}


static int i2c_wait_STOP(i2c_t *obj) {
    int timeout = 0;
    
    /* There is no timeout, but the upper limit value is set to avoid an infinite loop. */
    while (!(i2c_status(obj) & SR2_STOP)) {
        timeout ++;
        if (timeout >= TIMEOUT_1S) {
            return -1;
        }
    }

    return 0;
}

static void i2c_set_NACKF_STOP(i2c_t *obj) {
    /* SR2.NACKF = 0 */
    REG(SR2.UINT32) &= ~SR2_NACKF;
    /* SR2.STOP  = 0 */
    REG(SR2.UINT32) &= ~SR2_STOP;
}

static void i2c_set_err_noslave(i2c_t *obj) {
    i2c_stop(obj);
    (void)i2c_wait_STOP(obj);
    i2c_set_NACKF_STOP(obj);
}

static inline void i2c_power_enable(i2c_t *obj) {
    volatile uint8_t dummy;
    switch ((int)obj->i2c) {
        case I2C_0: CPGSTBCR9 &= ~(0x80); break;
        case I2C_1: CPGSTBCR9 &= ~(0x40); break;
        case I2C_2: CPGSTBCR9 &= ~(0x20); break;
        case I2C_3: CPGSTBCR9 &= ~(0x10); break;
    }
    dummy = CPGSTBCR9;
}

void i2c_init(i2c_t *obj, PinName sda, PinName scl) {
    /* determine the I2C to use */
    I2CName i2c_sda = (I2CName)pinmap_peripheral(sda, PinMap_I2C_SDA);
    I2CName i2c_scl = (I2CName)pinmap_peripheral(scl, PinMap_I2C_SCL);
    obj->i2c = pinmap_merge(i2c_sda, i2c_scl);
    MBED_ASSERT((int)obj->i2c != NC);

    /* enable power */
    i2c_power_enable(obj);

    /* set default frequency at 100k */
    i2c_frequency(obj, 100000);

    pinmap_pinout(sda, PinMap_I2C_SDA);
    pinmap_pinout(scl, PinMap_I2C_SCL);
}

inline int i2c_start(i2c_t *obj) {
    int timeout = 0;

    while (REG(CR2.UINT32) & CR2_BBSY) {
        timeout ++;
        if (timeout >= obj->bbsy_wait_cnt) {
            i2c_reg_reset(obj);
            /* Start Condition */
            REG(CR2.UINT8[0]) |= CR2_ST;
            return 0;
        }
    }
    /* Start Condition */
    REG(CR2.UINT8[0]) |= CR2_ST;

    return 0;
}

inline int i2c_stop(i2c_t *obj) {
    /* SR2.STOP  = 0 */
    REG(SR2.UINT32) &= ~SR2_STOP;
    /* Stop condition */
    REG(CR2.UINT32) |= CR2_SP;

    return 0;
}

static inline int i2c_do_write(i2c_t *obj, int value) {
    int timeout = 0;

    if (!(i2c_status(obj) & SR2_NACKF)) {
        /* RIICnSR2.NACKF=0 */
        /* There is no timeout, but the upper limit value is set to avoid an infinite loop. */
        while (!(i2c_status(obj) & SR2_TDRE)) {
            /* RIICnSR2.TDRE=0 */
            timeout ++;
            if (timeout >= TIMEOUT_1S) {
                return -1;
            }
            if (i2c_status(obj) & SR2_NACKF) {
                /* RIICnSR2.NACKF=1 */
                return -1;
            }
        }
        /* write the data */
        REG(DRT.UINT32) = value;
    } else {
        return -1;
    }

    return 0;
}

static inline int i2c_read_address_write(i2c_t *obj, int value) {
    int status;
    status = i2c_wait_TDRE(obj);
    if (status == 0) {
        /* write the data */
        REG(DRT.UINT32) = value;
        return 0;
    } else {
        return status;
    }

}

static inline int i2c_do_read(i2c_t *obj, int last) {
    if (last == 2) {
        /* this time is befor last byte read */
        /* Set MR3 WAIT bit is 1 */;
        REG(MR3.UINT32) |= MR3_WAIT;
    } else if (last == 1) {
        /* send a NOT ACK */
        REG(MR3.UINT32) |=  MR3_ACKWP;
        REG(MR3.UINT32) |=  MR3_ACKBT;
        REG(MR3.UINT32) &= ~MR3_ACKWP;
    } else {
        /* send a ACK */
        REG(MR3.UINT32) |=  MR3_ACKWP;
        REG(MR3.UINT32) &= ~MR3_ACKBT;
        REG(MR3.UINT32) &= ~MR3_ACKWP;
    }

    /* return the data */
    return (REG(DRR.UINT32) & 0xFF);
}

void i2c_frequency(i2c_t *obj, int hz) {
    FLOAT64_T pclk_val;
    FLOAT64_T wait_utime;
    volatile FLOAT64_T bps;
    volatile FLOAT64_T L_time;         /* H Width period */
    volatile FLOAT64_T H_time;         /* L Width period */
    uint32_t tmp_L_width;
    uint32_t tmp_H_width;
    uint32_t remainder;
    uint32_t wk_cks = 0;

    /* set PCLK */
    if (false == RZ_A1_IsClockMode0()) {
        pclk_val = (FLOAT64_T)CM1_RENESAS_RZ_A1_P0_CLK;
    } else {
        pclk_val = (FLOAT64_T)CM0_RENESAS_RZ_A1_P0_CLK;
    }

    /* Min 10kHz, Max 400kHz */
    if (hz < 10000) {
        bps = 10000;
    } else if (hz > 400000) {
        bps = 400000;
    } else {
        bps = (FLOAT64_T)hz;
    }

    /* Calculation L width time */
    L_time = (1 / (2 * bps));   /* Harf period of frequency */
    H_time = L_time;

    /* Check I2C mode of Speed */
    if (bps > 100000) {
        /* Fast-mode */
        L_time -= 102E-9;    /* Falling time of SCL clock. */
        H_time -= 138E-9;    /* Rising time of SCL clock. */
        /* Check L wideth */
        if (L_time < 1.3E-6) {
            /* Wnen L width less than 1.3us */
            /* Subtract Rise up and down time for SCL from H/L width */
            L_time = 1.3E-6;
            H_time = (1 / bps) - L_time - 138E-9 - 102E-9;
        }
    }

    tmp_L_width   = (uint32_t)(L_time * pclk_val * 10);
    tmp_L_width >>= 1;
    wk_cks++;
    while (tmp_L_width >= 341) {
        tmp_L_width >>= 1;
        wk_cks++;
    }
    remainder   = tmp_L_width % 10;
    tmp_L_width = ((tmp_L_width + 9) / 10) - 3;       /* carry */

    tmp_H_width   = (uint32_t)(H_time * pclk_val * 10);
    tmp_H_width >>= wk_cks;
    if (remainder == 0) {
        tmp_H_width   = ((tmp_H_width + 9) / 10) - 3; /* carry */
    } else {
        remainder    += tmp_H_width % 10;
        tmp_H_width   = (tmp_H_width / 10) - 3;
        if (remainder > 10) {
            tmp_H_width += 1;                         /* fine adjustment */
        }
    }
    /* timeout of BBSY bit is minimum low width by frequency */
    /* so timeout calculates "(low width) * 2" by frequency */
    wait_utime = (L_time * 2) * 1000000;
    /* 1 wait of BBSY bit is about 0.3us. if it's below 0.3us, wait count is set as 1. */
    if (wait_utime <= 0.3) {
        obj->bbsy_wait_cnt = 1;
    } else {
        obj->bbsy_wait_cnt = (int)(wait_utime / 0.3);
    }


    /* I2C Rate */
    obj->pclk_bit  = (uint8_t)(0x10 * wk_cks);        /* P_phi / xx */
    obj->width_low = (uint8_t)(tmp_L_width | 0x000000E0);
    obj->width_hi  = (uint8_t)(tmp_H_width | 0x000000E0);

    /* full reset */
    i2c_reg_reset(obj);
}

int i2c_read(i2c_t *obj, int address, char *data, int length, int stop) {
    int count = 0;
    int status;
    int value;
    volatile uint32_t work_reg = 0;

    status = i2c_start(obj);
    if (status != 0) {
        i2c_set_err_noslave(obj);
        return I2C_ERROR_BUS_BUSY;
    }
    /*  Send Slave address */
    status = i2c_read_address_write(obj, (address | 0x01));
    if (status != 0) {
        i2c_set_err_noslave(obj);
        return I2C_ERROR_NO_SLAVE;
    }
    /* wati RDRF */
    status = i2c_wait_RDRF(obj);
    /* check ACK/NACK */
    if ((status != 0) || (REG(SR2.UINT32) & CR2_NACKF == 1)) {
        /* Slave sends NACK */
        i2c_stop(obj);
        /* dummy read */
        value = REG(DRR.UINT32);
        (void)i2c_wait_STOP(obj);
        i2c_set_NACKF_STOP(obj);
        return I2C_ERROR_NO_SLAVE;
    }
    /* Read in all except last byte */
    if (length > 2) {
        /* dummy read */
        value = REG(DRR.UINT32);
        for (count = 0; count < (length - 1); count++) {
            /* wait for it to arrive */
            status = i2c_wait_RDRF(obj);
            if (status != 0) {
                i2c_set_err_noslave(obj);
                return I2C_ERROR_NO_SLAVE;
            }
            /* Recieve the data */
            if (count == (length - 2)) {
                value = i2c_do_read(obj, 1);
            } else if ((length >= 3) && (count == (length - 3))) {
                value = i2c_do_read(obj, 2);
            } else {
                value = i2c_do_read(obj, 0);
            }
            data[count] = (char)value;
        }
    } else if (length == 2) {
        /* Set MR3 WATI bit is 1 */;
        REG(MR3.UINT32) |= MR3_WAIT;
        /* dummy read */
        value = REG(DRR.UINT32);
        /* wait for it to arrive */
        status = i2c_wait_RDRF(obj);
        if (status != 0) {
            i2c_set_err_noslave(obj);
            return I2C_ERROR_NO_SLAVE;
        }
        /* send a NOT ACK */
        REG(MR3.UINT32) |=  MR3_ACKWP;
        REG(MR3.UINT32) |=  MR3_ACKBT;
        REG(MR3.UINT32) &= ~MR3_ACKWP;
        data[count] = (char)REG(DRR.UINT32);
        count++;
    } else if (length == 1) {
        /* Set MR3 WATI bit is 1 */;
        REG(MR3.UINT32) |=  MR3_WAIT;
        /* send a NOT ACK */
        REG(MR3.UINT32) |=  MR3_ACKWP;
        REG(MR3.UINT32) |=  MR3_ACKBT;
        REG(MR3.UINT32) &= ~MR3_ACKWP;
        /* dummy read */
        value = REG(DRR.UINT32);
    } else {
        return I2C_ERROR_NO_SLAVE;
    }
    /* wait for it to arrive */
    status = i2c_wait_RDRF(obj);
    if (status != 0) {
        i2c_set_err_noslave(obj);
        return I2C_ERROR_NO_SLAVE;
    }
    /* RIICnSR2.STOP = 0 */
    REG(SR2.UINT32) &= ~SR2_STOP;
    /* RIICnCR2.SP   = 1 */
    REG(CR2.UINT32) |= CR2_SP;
    /* RIICnDRR read */
    value = REG(DRR.UINT32) & 0xFF;
    data[count] = (char)value;
    /* RIICnMR3.WAIT = 0 */
    REG(MR3.UINT32) &= ~MR3_WAIT;
    (void)i2c_wait_STOP(obj);
    i2c_set_NACKF_STOP(obj);

    return length;
}

int i2c_write(i2c_t *obj, int address, const char *data, int length, int stop) {
    int cnt;
    int status;

    status = i2c_start(obj);
    if (status != 0) {
        i2c_set_err_noslave(obj);
        return I2C_ERROR_BUS_BUSY;
    }
    /*  Send Slave address */
    status = i2c_do_write(obj, address);
    if (status != 0) {
        i2c_set_err_noslave(obj);
        return I2C_ERROR_NO_SLAVE;
    }
    /* Send Write data */
    for (cnt=0; cnt<length; cnt++) {
        status = i2c_do_write(obj, data[cnt]);
        if(status != 0) {
            i2c_set_err_noslave(obj);
            return cnt;
        }
    }
    /* Wait send end */
    status = i2c_wait_TEND(obj);
    if (status != 0) {
        i2c_set_err_noslave(obj);
        return I2C_ERROR_NO_SLAVE;
    }
    i2c_stop(obj);
    (void)i2c_wait_STOP(obj);
    i2c_set_NACKF_STOP(obj);
    
    return length;
}

void i2c_reset(i2c_t *obj) {
    i2c_stop(obj);
    (void)i2c_wait_STOP(obj);
    i2c_set_NACKF_STOP(obj);
}

int i2c_byte_read(i2c_t *obj, int last) {
    int status;

    /* dummy read */
    (void)REG(DRR.UINT32);
    /* wait for it to arrive */
    status = i2c_wait_RDRF(obj);
    if (status != 0) {
        i2c_stop(obj);
        (void)i2c_wait_STOP(obj);
        i2c_set_NACKF_STOP(obj);
        return I2C_ERROR_NO_SLAVE;
    }
    
    return (i2c_do_read(obj, last) & 0xFF);
}

int i2c_byte_write(i2c_t *obj, int data) {
    int ack;
    int status = i2c_do_write(obj, (data & 0xFF));
    if (status != 0) {
        ack = 0;
    } else {
        ack = 1;
    }

    return ack;
}

void i2c_slave_mode(i2c_t *obj, int enable_slave) {
    if (enable_slave != 0) {
        REG(SER.UINT32)  =  0x01;  // only slave addr 1 is enabled
    } else {
        REG(SER.UINT32)  =  0x00;  // no slave addr enabled
    }
}

int i2c_slave_receive(i2c_t *obj) {
    int status;
    int retval;

    status = i2c_addressed(obj);
    switch(status) {
        case 0x3: retval = 1; break;
        case 0x2: retval = 2; break;
        case 0x1: retval = 3; break;
        default : retval = 1; break;
    }

    return(retval);
}

int i2c_slave_read(i2c_t *obj, char *data, int length) {
    int count = 0;
    int status;

    volatile int dummy = REG(DRR.UINT32) ;

    do {
        i2c_wait_RDRF(obj);
        status = i2c_status(obj);
        if(!(status & 0x10)) {
            data[count] = REG(DRR.UINT32) & 0xFF;
        }
        count++;
    } while ( !(status & 0x10)  && (count < length) );

    if(status & 0x10) {
        i2c_stop(obj);
        (void)i2c_wait_STOP(obj);
        i2c_set_NACKF_STOP(obj);
    }

    //i2c_clear_TDRE(obj);

    return count;
}

int i2c_slave_write(i2c_t *obj, const char *data, int length) {
    int count = 0;
    int status;

    if(length <= 0) {
        return(0);
    }

    do {
        status = i2c_do_write(obj, data[count]);
        count++;
    } while ((count < length) && !(status & 0x10));

    if (!(status & 0x10)) {
        i2c_stop(obj);
        (void)i2c_wait_STOP(obj);
        i2c_set_NACKF_STOP(obj);
    }

    i2c_clear_TDRE(obj);

    return(count);
}

void i2c_slave_address(i2c_t *obj, int idx, uint32_t address, uint32_t mask) {
    REG(SAR0.UINT32) = address & 0xfe;
}
