// SPDX-License-Identifier: GPL-2.0+
/**
 * Copyright (C) Shanghai FourSemi Semiconductor Co.,Ltd 2016-2024. All rights reserved.
 * 2024-03-25 File created.
 */

#include <linux/module.h>
#include "frsm-amp-drv.h"

extern struct i2c_driver frsm_i2c_driver;
extern struct platform_driver frsm_amp_driver;

static int __init frsm_drv_init(void)
{
	int ret;

	ret = platform_driver_register(&frsm_amp_driver);
	if (ret)
		pr_err("Failed to add frsm_amp_driver:%d\n", ret);

	return i2c_add_driver(&frsm_i2c_driver);
}

static void __exit frsm_drv_exit(void)
{
	platform_driver_unregister(&frsm_amp_driver);
	i2c_del_driver(&frsm_i2c_driver);
}

module_init(frsm_drv_init);
module_exit(frsm_drv_exit);

MODULE_AUTHOR("FourSemi SW <support@foursemi.com>");
MODULE_DESCRIPTION("ASoC FourSemi Audio Amplifier Driver");
MODULE_VERSION(FRSM_I2C_VERSION);
MODULE_LICENSE("GPL");
