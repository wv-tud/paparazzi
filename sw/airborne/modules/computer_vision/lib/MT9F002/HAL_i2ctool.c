#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <HAL_i2ctool.h>

#define I2C_MAX_RETRY 10
#define ARRAY_SIZE(_ar) (sizeof(_ar) / sizeof(_ar[0]))

int HAL_i2ctool_open(HAL_i2ctoolContext_t* ctx, char* path, int deviceAddr, int regAddrSize, int regSize)
{
	int res=0;

	// check ctx existence
	if (ctx == NULL)
	{
		res = -1;
	}

	// fill context
	if (!res)
	{
		ctx->i2cPath = path;
		ctx->deviceAddr = deviceAddr;
		ctx->regAddrSize = regAddrSize;
		ctx->regSize = regSize;
	}

	// open i2c interface
	if (!res)
	{
		ctx->fd = open(ctx->i2cPath, O_RDWR);
		if (ctx->fd < 0)
		{
		  fprintf(stderr,"Can't open %s: %s\n", ctx->i2cPath, strerror(errno));
		  res = -1;
		}
	}

	// configure I2C
	if (!res)
	{
		if (ioctl(ctx->fd, I2C_SLAVE_FORCE, ctx->deviceAddr) < 0)
		{
			perror("ioctl(I2C_SLAVE_FORCE) failed");
			res = -1;
		}
	}

	return res;
}

int HAL_i2ctool_write(HAL_i2ctoolContext_t* ctx, uint64_t regaddr, uint64_t val)
{
	/* I assume the max register address/size is 64 bits */
	uint8_t   buf[8 + 8];
	uint8_t  *pb;
	unsigned  msgsz = ctx->regAddrSize + ctx->regSize;
	int       i;

	pb = buf;

	for (i = ctx->regAddrSize - 1; i >= 0; i--)
		*pb++ = (regaddr >> (i * 8)) & 0xff;

	for (i = ctx->regSize - 1; i >= 0; i--)
		*pb++ = (val >> (i * 8)) & 0xff;

	//fprintf(stderr,"writing 0x%02x-0x%llx: 0x%llx\n", ctx->deviceAddr, regaddr, val);
  int retry=I2C_MAX_RETRY;
  int res=0;
  while(retry--)
  {
    res = write(ctx->fd, buf, msgsz);
    if (res == (int)msgsz)
    {
      break;
    }
  }
  if (res != (int)msgsz)
  {
		perror("register write failed");
		return -1;
	}

	return 0;
}

int HAL_i2ctool_writeList(HAL_i2ctoolContext_t* ctx, HAL_i2ctool_regVal_list_t* regList)
{
	uint32_t i;
	if (ctx == NULL || regList == NULL || regList->list == NULL)
	{
		fprintf(stderr,"%s:%d error, check parameters\n",__FILE__,__LINE__);
		return -1;
	}

	for (i=0; i<regList->listSize; i++)
	{
		if (HAL_i2ctool_write(ctx, regList->list[i].reg, regList->list[i].val))
		{
			return -1;
		}
	}
	return 0;
}

int HAL_i2ctool_read(HAL_i2ctoolContext_t* ctx, uint64_t regaddr, uint64_t* val)
{
	/* I assume the max register address/size is 64 bits */
	uint8_t regaddr_bytes[8];
	uint8_t reg_bytes[8];
	uint8_t *pb;
	int     i;

	struct i2c_msg msgs[2] = {
			[0] = {
					.addr = ctx->deviceAddr,
					.flags = 0,
					.len = ctx->regAddrSize,
					.buf = regaddr_bytes,
			},
			[1] = {
					.addr = ctx->deviceAddr,
					.flags = I2C_M_RD,
					.len = ctx->regSize,
					.buf = reg_bytes,
			}
	};

	struct i2c_rdwr_ioctl_data i2c_rdwr = {
			.msgs = msgs,
			.nmsgs = ARRAY_SIZE(msgs),
	};

	pb = regaddr_bytes;

	for (i = ctx->regAddrSize - 1; i >= 0; i--)
		*pb++ = (regaddr >> (i * 8)) & 0xff;

	memset(reg_bytes, 0, sizeof(reg_bytes));

	//fprintf(stderr,"reading 0x%02x-0x%llx: ", ctx->deviceAddr, regaddr);
	//fflush(stdout);
	if (ioctl(ctx->fd, I2C_RDWR, &i2c_rdwr) < 0) {
		perror("ioctl(I2C_RDWR) failed");
		fprintf(stderr,"error\n");
		return -1;
	}

	// set val
	*val=0;
	for (i = ctx->regSize - 1; i >= 0; i--)
		((uint8_t*)val)[i] = reg_bytes[ctx->regSize - 1 - i];

	 /*
	 fprintf(stderr,"0x");
	  for (i = 0; i < ctx->regSize; i++)
	    fprintf(stderr,"%02x", reg_bytes[i]);
	  fprintf(stderr,"\n");
	*/
	return 0;
}

/*
 * Write a value to a register with a different register size from what has been set with i2ctool_open()
 * @param ctx       i2c context
 * @param regaddr   register address
 * @param val       value to write
 * @param valSize   register size in byte  (max 8)
 * @return            0: success
 *                   -1: failure
 */
int HAL_i2ctool_write_regSize(HAL_i2ctoolContext_t* ctx, uint64_t regaddr, uint64_t val, uint32_t regSize)
{
  /* I assume the max register address/size is 64 bits */
  uint8_t   buf[8 + 8];
  uint8_t  *pb;
  unsigned  msgsz = ctx->regAddrSize + regSize;
  int       i;

  pb = buf;

  for (i = ctx->regAddrSize - 1; i >= 0; i--)
    *pb++ = (regaddr >> (i * 8)) & 0xff;

  for (i = regSize - 1; i >= 0; i--)
    *pb++ = (val >> (i * 8)) & 0xff;

  //fprintf(stderr,"writing 0x%02x-0x%llx: 0x%llx\n", ctx->deviceAddr, regaddr, val);
  int retry=I2C_MAX_RETRY;
  int res=0;
  while(retry--)
  {
    res = write(ctx->fd, buf, msgsz);
    if (res == (int)msgsz)
    {
      break;
    }
  }
  if (res != (int)msgsz)
  {
    perror("register write failed");
    return -1;
  }

  return 0;
}

/*
 * Read a register with a different register size from what has been set with i2ctool_open()
 * @param ctx       i2c context
 * @param regaddr   register address
 * @param val       register value result
 * @param regSize   register size in byte  (max 8)
 * @return            0: success
 *                   -1: failure
 */
int HAL_i2ctool_read_regSize(HAL_i2ctoolContext_t* ctx, uint64_t regaddr, uint64_t* val, uint32_t regSize)
{
  /* I assume the max register address/size is 64 bits */
  uint8_t regaddr_bytes[8];
  uint8_t reg_bytes[8];
  uint8_t *pb;
  int     i;

  struct i2c_msg msgs[2] = {
      [0] = {
          .addr = ctx->deviceAddr,
          .flags = 0,
          .len = ctx->regAddrSize,
          .buf = regaddr_bytes,
      },
      [1] = {
          .addr = ctx->deviceAddr,
          .flags = I2C_M_RD,
          .len = regSize,
          .buf = reg_bytes,
      }
  };

  struct i2c_rdwr_ioctl_data i2c_rdwr = {
      .msgs = msgs,
      .nmsgs = ARRAY_SIZE(msgs),
  };

  pb = regaddr_bytes;

  for (i = ctx->regAddrSize - 1; i >= 0; i--)
    *pb++ = (regaddr >> (i * 8)) & 0xff;

  memset(reg_bytes, 0, sizeof(reg_bytes));

  //fprintf(stderr,"reading 0x%02x-0x%llx: ", ctx->deviceAddr, regaddr);
  //fflush(stdout);
  if (ioctl(ctx->fd, I2C_RDWR, &i2c_rdwr) < 0) {
    perror("ioctl(I2C_RDWR) failed");
    fprintf(stderr,"error\n");
    return -1;
  }

  // set val
  *val=0;
  for (i = regSize - 1; i >= 0; i--)
    ((uint8_t*)val)[i] = reg_bytes[regSize - 1 - i];

   /*
   fprintf(stderr,"0x");
    for (i = 0; i < ctx->regSize; i++)
      fprintf(stderr,"%02x", reg_bytes[i]);
    fprintf(stderr,"\n");
  */
  return 0;
}
