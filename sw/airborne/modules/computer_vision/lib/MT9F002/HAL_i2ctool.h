#ifndef _HAL_I2CTOOL_H_
#define _HAL_I2CTOOL_H_
#include <stdint.h>

typedef struct HAL_i2ctoolContext_t_
{
	char * i2cPath;
	int fd;
	int deviceAddr;
	int regAddrSize;
	int regSize;
}HAL_i2ctoolContext_t;

typedef struct HAL_i2ctool_regVal_t_
{
	uint64_t reg;
	uint64_t val;
}HAL_i2ctool_regVal_t;

typedef struct HAL_i2ctool_regVal_list_t_
{
	uint32_t listSize;
	HAL_i2ctool_regVal_t* list;
}HAL_i2ctool_regVal_list_t;

/*
 * Open an i2c context
 * @param ctx         context to initialize
 * @param path        path to the i2c interface
 * @param deviceAddr  i2c addr of the device
 * @param regAddrSize register address size in byte (max 8)
 * @param regSize     register size in byte (max 8)
 * @return            0: success
 *                   -1: failure
 */
int HAL_i2ctool_open(HAL_i2ctoolContext_t* ctx, char* path, int deviceAddr, int regAddrSize, int regSize);

/*
 * Write a value to a register
 * @param ctx       i2c context
 * @param regaddr   register address
 * @param val       value to write
 * @return            0: success
 *                   -1: failure
 */
int HAL_i2ctool_write(HAL_i2ctoolContext_t* ctx, uint64_t regaddr, uint64_t val);

/*
 * Read a register
 * @param ctx       i2c context
 * @param regaddr   register address
 * @param val       register value result
 * @return            0: success
 *                   -1: failure
 */
int HAL_i2ctool_read(HAL_i2ctoolContext_t* ctx, uint64_t regaddr, uint64_t* val);

/*
 * Write a value to a register with a different register size from what has been set with i2ctool_open()
 * @param ctx       i2c context
 * @param regaddr   register address
 * @param val       value to write
 * @param valSize   register size in byte  (max 8)
 * @return            0: success
 *                   -1: failure
 */
int HAL_i2ctool_write_regSize(HAL_i2ctoolContext_t* ctx, uint64_t regaddr, uint64_t val, uint32_t regSize);

/*
 * Read a register with a different register size from what has been set with i2ctool_open()
 * @param ctx       i2c context
 * @param regaddr   register address
 * @param val       register value result
 * @param regSize   register size in byte  (max 8)
 * @return            0: success
 *                   -1: failure
 */
int HAL_i2ctool_read_regSize(HAL_i2ctoolContext_t* ctx, uint64_t regaddr, uint64_t* val, uint32_t regSize);

int HAL_i2ctool_writeList(HAL_i2ctoolContext_t* ctx, HAL_i2ctool_regVal_list_t* regList);
#endif
