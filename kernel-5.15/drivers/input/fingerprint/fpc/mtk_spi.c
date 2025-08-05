/*
 * FPC Fingerprint sensor device driver
 *
 * This driver will control the platform resources that the FPC fingerprint
 * sensor needs to operate. The major things are probing the sensor to check
 * that it is actually connected and let the Kernel know this and with that also
 * enabling and disabling of regulators, enabling and disabling of platform
 * clocks.
 * *
 * The driver will expose most of its available functionality in sysfs which
 * enables dynamic control of these features from eg. a user space process.
 *
 * The sensor's IRQ events will be pushed to Kernel's event handling system and
 * are exposed in the drivers event node. This makes it possible for a user
 * space process to poll the input node and receive IRQ events easily. Usually
 * this node is available under /dev/input/eventX where 'X' is a number given by
 * the event system. A user space process will need to traverse all the event
 * nodes and ask for its parent's name (through EVIOCGNAME) which should match
 * the value in device tree named input-device-name.
 *
 *
 * Copyright (c) 2018 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/pm_wakeup.h>
#include <linux/regulator/consumer.h>
#include <linux/proc_fs.h>
#include <linux/clk.h>

#define USE_REGULATOR  1

#ifdef FPC_DRIVER_FOR_ISEE
#include "teei_fp.h"
#include "tee_client_api.h"
#endif
#define SAMSUNG_FP_SUPPORT   1

#ifdef SAMSUNG_FP_SUPPORT
#define FINGERPRINT_ADM_CLASS_NAME "fingerprint"
#endif

static int fpc_fp_adm = 0;
static int fpc_adm_release =0;

#define PROC_NAME  "fp_info"
static int fp_info_flag = 0;
static int regulator_flag = 0;
static int g_broken_flag = 0;
static struct proc_dir_entry *proc_entry;
#ifdef FPC_DRIVER_FOR_ISEE
struct TEEC_UUID uuid_ta_fpc = { 0x7778c03f, 0xc30c, 0x4dd0,
	{0xa3, 0x19, 0xea, 0x29, 0x64, 0x3d, 0x4d, 0x4b}
};
#endif

#define FPC1022_CHIP 0x1000
#define FPC1022_CHIP_MASK_SENSOR_TYPE 0xf000
#define FPC_POWEROFF_SLEEP_US 1000
#define FPC_RESET_LOW_US 6000
#define FPC_RESET_HIGH1_US 100
#define FPC_RESET_HIGH2_US 5000
#define FPC_TTW_HOLD_TIME 1000
#define FPC102X_REG_HWID 252
#define MAX_RESET_NUM 4
u32 spi_speed = 1 * 1000000;

#define FPC_CONFIG_COMPAT    1
//#define PIN_CONTROL
#ifdef SAMSUNG_FP_SUPPORT
static ssize_t adm_show(struct device *dev,struct device_attribute *attr,char *buf)
{
  return snprintf(buf,3,"1\n");
}

static DEVICE_ATTR(adm, 0664, adm_show, NULL);
struct class *fingerprint_adm_class;
struct device *fingerprint_adm_dev;
#endif

#ifdef PIN_CONTROL
static const char * const pctl_names[] = {
    "fpsensor_fpc_rst_low",
	"fpsensor_fpc_rst_high",
	"fpsensor_fpc_power_high",
	"fpsensor_fpc_power_low",
};
#endif

#define GET_STR_BYSTATUS(status) (status==0? "Success":"Failure")
#define ERROR_LOG       (0)
#define INFO_LOG        (1)
#define DEBUG_LOG       (2)

/* debug log setting */
static u8 debug_level = DEBUG_LOG;
#define fpsensor_log(level, fmt, args...) do { \
    if (debug_level >= level) {\
        if( level == DEBUG_LOG) { \
            printk("fingerprint %s +%d, " fmt, __func__, __LINE__, ##args); \
        } else { \
            printk("fingerprint: " fmt, ##args); \
        }\
    } \
} while (0)
#define func_entry()    do{fpsensor_log(DEBUG_LOG, "%s, %d, entry\n", __func__, __LINE__);}while(0)
#define func_exit()     do{fpsensor_log(DEBUG_LOG, "%s, %d, exit\n", __func__, __LINE__);}while(0)

#ifdef USE_REGULATOR
struct vreg_config {
	char *name;
	unsigned long vmin;
	unsigned long vmax;
	int ua_load;
};

static const struct vreg_config vreg_conf[] = {
	{ "vdd_io", 3300000UL, 3300000UL, 6000, },
};
#endif

struct fpc_data {
	struct device *dev;
	struct spi_device *spidev;
#ifdef USE_REGULATOR
		struct regulator *vreg[ARRAY_SIZE(vreg_conf)];
#endif
#ifdef PIN_CONTROL
	struct pinctrl *pinctrl_fpc;
	struct pinctrl_state *pinctrl_state[ARRAY_SIZE(pctl_names)];
#endif
	int irq_gpio;
	int rst_gpio;
//	int power_ctl_gpio;
	bool wakeup_enabled;
	struct wakeup_source *ttw_wl;

#ifdef FPC_CONFIG_COMPAT
    bool compatible_enabled;
#endif
};

static DEFINE_MUTEX(spidev_set_gpio_mutex);

extern void mt_spi_disable_master_clk(struct spi_device *spidev);
extern void mt_spi_enable_master_clk(struct spi_device *spidev);
bool fpc1022_fp_exist = false;

bool g_fpsensor_clocks_enabled;
struct spi_device *g_fpsensor_spidev;
static int reset_num = 0;
static irqreturn_t fpc_irq_handler(int irq, void *handle);

static int proc_show_ver(struct seq_file *file,void *v)
{
	seq_printf(file,"[Fingerprint]:fpc_fp  [IC]:fpc1560\n");
	return 0;
}

static int proc_open(struct inode *inode,struct file *file)
{
	printk("fpc proc_open\n");
	single_open(file,proc_show_ver,NULL);
	return 0;
}

static const struct proc_ops proc_file_fpc_ops = {
	.proc_open = proc_open,
	.proc_read = seq_read,
	.proc_release = single_release,
};
#ifdef USE_REGULATOR
static int vreg_setup(struct fpc_data *fpc1020, const char *name,
	bool enable)
{
	size_t i;
	int rc;
	struct regulator *vreg;
	struct device *dev = fpc1020->dev;

	for (i = 0; i < ARRAY_SIZE(fpc1020->vreg); i++) {
		const char *n = vreg_conf[i].name;

		if (!strncmp(n, name, strlen(n)))
			goto found;
	}

	dev_err(dev, "fpc_regulator %s not found : %d\n", name, enable);

	return -EINVAL;

found:
	vreg = fpc1020->vreg[i];
	dev_err(dev, "fpc_regulator_power %s found, enable: %d regulator_flag:%d\n", name, enable, regulator_flag);
	if (enable && !regulator_flag) {
		if (!vreg) {
			vreg = devm_regulator_get(dev, name);
			if (IS_ERR_OR_NULL(vreg)) {
				dev_err(dev, "fpc_regulator Unable to get %s\n", name);
				return PTR_ERR(vreg);
			}
		}

		if (regulator_count_voltages(vreg) > 0) {
			rc = regulator_set_voltage(vreg, vreg_conf[i].vmin,
					vreg_conf[i].vmax);
			if (rc){
				dev_err(dev,
					"fpc_regulator Unable to set voltage on %s, %d\n",
			name, rc);
				return -1;
			}else{
				printk("fpc_regulator set voltage sucess\n");
			}
		}

		//rc = regulator_set_load(vreg, vreg_conf[i].ua_load);
		//if (rc < 0){
		//	dev_err(dev, "fpc_regulator Unable to set current on %s, %d\n",
		//			name, rc);
		//	return -1;
		//}else {
		//	printk("fpc_regulator to set current success\n");
		//}

		rc = regulator_enable(vreg);
		if (rc) {
			dev_err(dev, "fpc_regulator error enabling %s: %d\n", name, rc);
			vreg = NULL;
		}else{
			regulator_flag = 1;
		}
		fpc1020->vreg[i] = vreg;
	} else {
		if(regulator_flag){
	        dev_err(dev, "fpc_regulator_power %s found, enable: %d regulator_flag:%d\n", name, enable, regulator_flag);
			if (vreg) {
				if (regulator_is_enabled(vreg)) {
					printk("regulator_is_enabled vreg ok \n");
					regulator_disable(vreg);
					regulator_put(vreg);
					dev_err(dev, "fpc_regulator disabled %s\n", name);
					regulator_flag = 0;
				}
				fpc1020->vreg[i] = NULL;
			}
		}else {
			printk("fpc_regulator do nothing since no enable by fpc\n");
		}
		rc = 0;
	}
	return rc;
}

#endif

static int set_clks(bool enable)
{
	int rc = 0;
	if(g_fpsensor_clocks_enabled == enable)
		return rc;
	if (enable) {
		mt_spi_enable_master_clk(g_fpsensor_spidev);
		g_fpsensor_clocks_enabled = true;
		rc = 1;
	} else {
		mt_spi_disable_master_clk(g_fpsensor_spidev);
		g_fpsensor_clocks_enabled = false;
		rc = 0;
	}

	return rc;
}

static int fpc_dts_pin_init( struct fpc_data * fpc)
{
	struct device_node *node = NULL;
    struct device *dev = &fpc->spidev->dev;
    int ret = 0;

	// request reset
	node = of_find_compatible_node(NULL, NULL, "mediatek,fpsensor");
	if (node == NULL) {
		ret = -EINVAL;
		fpsensor_log(ERROR_LOG, "%s cannot find node.\n", __func__);
		set_clks(false);
		return ret;
	}
	fpc->rst_gpio = of_get_named_gpio(node, "fpc,rst_gpio", 0);

	if (fpc->rst_gpio < 0) {
	     fpsensor_log(ERROR_LOG,"%s, failed to get fpc,rst_gpio!\n", __func__);
	     return fpc->rst_gpio;
	 }
	ret = devm_gpio_request(dev, fpc->rst_gpio, "fpc,rst_gpio");
	if (ret) {
	     fpsensor_log(ERROR_LOG,"%s, failed to request rst_gpio %d\n", __func__, fpc->rst_gpio);
	     gpio_free(fpc->rst_gpio);
	     ret = devm_gpio_request(dev, fpc->rst_gpio, "fpc,rst_gpio");
	     if(ret){
	         fpsensor_log(ERROR_LOG,"%s failed to request rst_gpio[%d], ret = %d. retry failed.\n", __func__, fpc->rst_gpio, ret);
	         return ret;
	     }
	     fpsensor_log(INFO_LOG,"%s, the return value of request fpc->rst_gpio is %d\n", __func__, ret);
	}
	
	// request irq
	fpc->irq_gpio = of_get_named_gpio(node, "fpc,irq_gpio", 0); 
    if (fpc->irq_gpio < 0) {
        fpsensor_log(ERROR_LOG,"%s failed to get fpc,irq_gpio!\n", __func__);
        return fpc->irq_gpio;
    } 
    fpsensor_log(INFO_LOG, "%s, irq_gpio=%d\n", __func__, fpc->irq_gpio);
    ret = devm_gpio_request(dev, fpc->irq_gpio, "fpc,irq_gpio");
    if (ret) {
        fpsensor_log(ERROR_LOG,"%s failed to request irq_gpio %d, ret = %d.\n", __func__, fpc->irq_gpio, ret);
        gpio_free(fpc->irq_gpio);
        ret = devm_gpio_request(dev, fpc->irq_gpio, "fpc,irq_gpio");
		if(ret){
			fpsensor_log(ERROR_LOG,"%s failed to request irq_gpio %d, ret = %d.retry failed.\n", __func__, fpc->irq_gpio, ret);
			return ret;
		}
    }
	fpsensor_log(INFO_LOG,"%s the return value of request fpc->irq_gpio is %d\n", __func__, fpc->irq_gpio);
	
	return ret;
}


#ifdef PIN_CONTROL
static int select_pin_ctl(struct fpc_data *fpc, const char *name)
{
	size_t i;
	int rc;
    fpsensor_log(INFO_LOG, "fpc %s entry.\n", __func__);
	for (i = 0; i < ARRAY_SIZE(fpc->pinctrl_state); i++) {
		const char *n = pctl_names[i];
		if (!strncmp(n, name, strlen(n))) {
			mutex_lock(&spidev_set_gpio_mutex);
			rc = pinctrl_select_state(fpc->pinctrl_fpc, fpc->pinctrl_state[i]);
			mutex_unlock(&spidev_set_gpio_mutex);
			if (rc)
				fpsensor_log(ERROR_LOG, "fpc %s cannot select '%s'\n", __func__, name);
			else
				fpsensor_log(ERROR_LOG, "fpc %s selected '%s'\n", __func__, name);
			goto exit;
		}
	}
	rc = -EINVAL;
	fpsensor_log(INFO_LOG, "fpc %s:'%s' found.\n", __func__, name);
exit:
	return rc;
}
#endif

static int hw_reset(struct  fpc_data *fpc)
{
    int rc = 0;
    fpsensor_log(INFO_LOG, "fpc %s entry.\n", __func__);
    
#ifdef PIN_CONTROL
	rc = select_pin_ctl(fpc, "fpsensor_fpc_rst_high");
    if(rc){
		fpsensor_log(ERROR_LOG, "fpc %s pin ctrl reset active failed.\n", __func__);
		return rc;
	}
	usleep_range(FPC_RESET_HIGH1_US, FPC_RESET_HIGH1_US + 100);

	rc = select_pin_ctl(fpc, "fpsensor_fpc_rst_low");
    if(rc){
		fpsensor_log(ERROR_LOG, "fpc %s pin ctrl reset reset failed.\n", __func__);
		return rc;
	}
	usleep_range(FPC_RESET_LOW_US, FPC_RESET_LOW_US + 100);

	rc = select_pin_ctl(fpc, "fpsensor_fpc_rst_high");
    if(rc){
		fpsensor_log(ERROR_LOG, "fpc %s pin ctrl reset active failed.\n", __func__);
		return rc;
	}
	usleep_range(FPC_RESET_HIGH2_US, FPC_RESET_HIGH2_US + 100);
#else
    gpio_direction_output(fpc->rst_gpio, 1);
	usleep_range(FPC_RESET_HIGH1_US, FPC_RESET_HIGH1_US + 100);
	gpio_direction_output(fpc->rst_gpio, 0);
	usleep_range(FPC_RESET_LOW_US, FPC_RESET_LOW_US + 100);
	gpio_direction_output(fpc->rst_gpio, 1);
	usleep_range(FPC_RESET_HIGH2_US, FPC_RESET_HIGH2_US + 100);
#endif

	func_exit();
	return rc;
}
static int hw_power_reset(struct fpc_data *fpc, int enable)
{
	int ret = 0;
	fpsensor_log(INFO_LOG, "fpc %s entry.\n", __func__);

#ifdef USE_REGULATOR
	fpsensor_log(INFO_LOG, "fpc %s use regulator.\n", __func__);
	set_clks(false);
	ret = vreg_setup(fpc, "vdd_io", false);
	if (ret != 0 ){
		fpsensor_log(ERROR_LOG, "fpc %s vreg_setup:false failed!\n", __func__);
		return ret;
	}
	fpsensor_log(ERROR_LOG, "fpc %s vreg_setup:false success!\n", __func__);
	usleep_range(FPC_POWEROFF_SLEEP_US, FPC_POWEROFF_SLEEP_US + 100);

    if(enable) {
        ret = vreg_setup(fpc, "vdd_io", true);
	    if (ret !=0 ){
		    fpsensor_log(ERROR_LOG, "fpc %s vreg_setup:true failed!\n", __func__);
		    return ret;
	    }
		fpsensor_log(ERROR_LOG, "fpc %s vreg_setup:true sucess!\n", __func__);
	    usleep_range(FPC_POWEROFF_SLEEP_US, FPC_POWEROFF_SLEEP_US + 100);
	    set_clks(true);
	    (void)hw_reset(fpc);
    }
	
#else
	if(gpio_is_valid(fpc->power_ctl_gpio)){

        fpsensor_log(INFO_LOG, "fpc %s POWER GPIO#%d.\n", __func__, fpc->power_ctl_gpio);
		set_clks(false);
        
#ifdef PIN_CONTROL       
        ret = select_pin_ctl(fpc, "fpsensor_fpc_power_low");
        if(ret){
            fpsensor_log(ERROR_LOG, "fpc %s pin ctrl power deactive failed.\n", __func__);
            return ret;
        }
        usleep_range(FPC_POWEROFF_SLEEP_US, FPC_POWEROFF_SLEEP_US + 100);
        if(enable){
            ret = select_pin_ctl(fpc, "fpsensor_fpc_power_high");
            if(ret){
                fpsensor_log(ERROR_LOG, "fpc %s pin ctrl power active failed.\n", __func__);
                return ret;
            }
            usleep_range(FPC_POWEROFF_SLEEP_US, FPC_POWEROFF_SLEEP_US + 100);
            set_clks(true);
            (void)hw_reset(fpc);
        }
#else
		gpio_direction_output(fpc->power_ctl_gpio, 0);
		usleep_range(FPC_POWEROFF_SLEEP_US, FPC_POWEROFF_SLEEP_US + 100);
        if(enable){
            gpio_direction_output(fpc->power_ctl_gpio, 1);
            usleep_range(FPC_POWEROFF_SLEEP_US, FPC_POWEROFF_SLEEP_US + 100);
            set_clks(true);
            (void)hw_reset(fpc);
        }
#endif
	}
	else{
        fpsensor_log(ERROR_LOG, "fpc %s fpc POWER GPIO[%d] isn't valid.\n", __func__, fpc->power_ctl_gpio);
		ret = -1;
	}
#endif
    func_exit();
	return ret;
}
#ifdef DEBUG_SPI
static int spi_read_hwid(struct spi_device *spi, u8 * rx_buf)
{
	int error;
	struct spi_message msg;
	struct spi_transfer *xfer;
	u8 tmp_buf[10] = { 0 };
	u8 addr = FPC102X_REG_HWID;
    fpsensor_log(INFO_LOG, "fpc %s entry.\n", __func__);
    
	xfer = kzalloc(sizeof(*xfer) * 2, GFP_KERNEL);
	if (xfer == NULL) {
        fpsensor_log(ERROR_LOG, "fpc %s no memory for SPI transfer.\n", __func__);
		return -ENOMEM;
	}

	spi_message_init(&msg);

	tmp_buf[0] = addr;
	xfer[0].tx_buf = tmp_buf;
	xfer[0].len = 1;
	xfer[0].delay_usecs = 5;

#ifdef CONFIG_SPI_MT65XX
	xfer[0].speed_hz = spi_speed;
    fpsensor_log(DEBUG_LOG, "fpc %s %d, now spi-clock:%d.\n", __func__, __LINE__, xfer[0].speed_hz);
#endif

	spi_message_add_tail(&xfer[0], &msg);

	xfer[1].tx_buf = tmp_buf + 2;
	xfer[1].rx_buf = tmp_buf + 4;
	xfer[1].len = 2;
	xfer[1].delay_usecs = 5;

#ifdef CONFIG_SPI_MT65XX
	xfer[1].speed_hz = spi_speed;
#endif

	spi_message_add_tail(&xfer[1], &msg);
	error = spi_sync(spi, &msg);
	if (error)
        fpsensor_log(ERROR_LOG, "fpc %s :spi_sync failed.\n", __func__);
	memcpy(rx_buf, (tmp_buf + 4), 2);

	kfree(xfer);
	xfer = NULL;

	return 0;
}
static int check_hwid(struct spi_device *spi)
{
	int error;
	u32 time_out = 0;
	u8 tmp_buf[2] = { 0 };
	u16 hardware_id;
    fpsensor_log(INFO_LOG, "fpc %s entry.\n", __func__);
    
	do {
		spi_read_hwid(spi, tmp_buf);
        fpsensor_log(INFO_LOG, "fpc %s, chip version is 0x%x, 0x%x\n", __func__, tmp_buf[0], tmp_buf[1]);
		time_out++;

		hardware_id = ((tmp_buf[0] << 8) | (tmp_buf[1]));
        fpsensor_log(INFO_LOG, "fpc %s, fpc hardware_id[0]= 0x%x id[1]=0x%x\n", __func__, tmp_buf[0], tmp_buf[1]);
		if ( hardware_id > 0x1800) {
			pr_err("fpc match hardware_id = 0x%x is true\n",
			       hardware_id);
            fpsensor_log(INFO_LOG, "fpc %s, match hardware_id = 0x%x is true.\n", __func__, hardware_id);
			error = 0;
		} else {
			fpsensor_log(INFO_LOG, "fpc %s, match hardware_id = 0x%x is failed.\n", __func__, hardware_id);
			error = -1;
		}

		if (!error) {
			fpsensor_log(ERROR_LOG, "fpc %s, fpc1553_S160 chip version check pass, time_out=%d\n",
			       __func__, time_out);
			return 0;
		}
	} while (time_out < 2);

	fpsensor_log(ERROR_LOG, "%s, fpc chip version read failed, time_out=%d\n",
	       __func__, time_out);

	return -1;
}
#endif
static int fpc_power_supply(struct fpc_data *fpc)
{
	int ret = 0;	
//	struct device *dev = &fpc->spidev->dev;
//	struct device_node *node = of_find_compatible_node(NULL, NULL, "mediatek,fpsensor");

	func_entry();
#ifdef USE_REGULATOR
	ret = vreg_setup(fpc, "vdd_io", true);
	if (ret != 0){
		fpsensor_log(ERROR_LOG, "fpc_regulator %s vreg_setup:false failed!\n", __func__);
		return ret;
	}
#else
	if(!node){
		fpsensor_log(ERROR_LOG, "%s, no of node found\n", __func__);
		return -EINVAL;
	}

	fpc->power_ctl_gpio = of_get_named_gpio(node, "fpc,power_ctl_gpio", 0);

	if (fpc->power_ctl_gpio < 0) {
        fpsensor_log(ERROR_LOG,"%s, failed to get fpc,power_ctl_gpio!\n", __func__);
        return fpc->power_ctl_gpio;
    }
	ret = devm_gpio_request(dev, fpc->power_ctl_gpio, "fpc,power_ctl_gpio");
    if (ret) {
        fpsensor_log(ERROR_LOG,"%s, failed to request power_ctl_gpio %d\n", __func__, fpc->power_ctl_gpio);
        // goto dts_init_exit;
        gpio_free(fpc->power_ctl_gpio);
        ret = devm_gpio_request(dev, fpc->power_ctl_gpio, "fpc,power_ctl_gpio");
        if(ret){
            fpsensor_log(ERROR_LOG,"%s failed to request power_ctl_gpio[%d], ret = %d. retry failed.\n", __func__, fpc->power_ctl_gpio, ret);
            return ret;
        }
        fpsensor_log(INFO_LOG,"%s, the return value of request fpc->power_ctl_gpio is %d\n", __func__, ret);
   }
	
#ifdef PIN_CONTROL
    ret = select_pin_ctl(fpc, "fpsensor_fpc_power_high");
    if(ret){
        fpsensor_log(ERROR_LOG, "fpc %s pin ctrl fpsensor_fpc_power_high failed.\n", __func__);
        return ret;
    }
#else
  //  gpio_direction_output(fpc->power_ctl_gpio, 1);
#endif	
#endif
 //  fpsensor_log(INFO_LOG, "%s, the value after set fpc->power_ctl_gpio 1 is %d\n", __func__, gpio_get_value(fpc->power_ctl_gpio));
   func_exit();
   return ret;
}


static ssize_t hw_reset_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    int rc;
	struct  fpc_data *fpc = dev_get_drvdata(dev);
    fpsensor_log(INFO_LOG, "fpc %s entry reset_num:%d. cmd:%s \n", __func__, reset_num, buf);
	if (!strncmp(buf, "reset", strlen("reset"))) {
        //if(reset_num > MAX_RESET_NUM){
           // fpsensor_log(INFO_LOG, "fpc %s hardware abnormal, shut down.\n", __func__);
           // rc = hw_power_reset(fpc,0);
           // reset_num = 0;
           // return rc ? rc : count;
       // }
        reset_num++;
		//rc = hw_power_reset(fpc,1);
	    rc = vreg_setup(fpc, "vdd_io", false);
		return rc ? rc : count;
	}
    else if (!strncmp(buf, "clear", strlen("clear"))) {
        reset_num = 0;
        g_broken_flag = 1;
        return count;
    }
	else
		return -EINVAL;
}
static DEVICE_ATTR(hw_reset, S_IWUSR, NULL, hw_reset_set);
	
/**
 * sysfs node to enable/disable (power up/power down) the touch sensor
 *
 * @see device_prepare
 */
static ssize_t device_prepare_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct fpc_data *fpc = dev_get_drvdata(dev);
	fpsensor_log(INFO_LOG, "fpc %s entry cmd:%s \n", __func__, buf);
	if (!strncmp(buf, "enable", strlen("enable")))
		rc = hw_power_reset(fpc, 1);
	else if (!strncmp(buf, "disable", strlen("disable")))
		rc = hw_power_reset(fpc, 0);
	else
		return -EINVAL;

	return rc ? rc : count;
}
static DEVICE_ATTR(device_prepare, S_IWUSR, NULL, device_prepare_set);

/**
 * sysfs node for controlling whether the driver is allowed
 * to wake up the platform on interrupt.
 */
static ssize_t wakeup_enable_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct  fpc_data *fpc = dev_get_drvdata(dev);
	ssize_t ret = count;
    fpsensor_log(INFO_LOG, "fpc %s entry.\n", __func__);
	if (!strncmp(buf, "enable", strlen("enable"))) {
		fpc->wakeup_enabled = true;
		smp_wmb();
	} else if (!strncmp(buf, "disable", strlen("disable"))) {
		fpc->wakeup_enabled = false;
		smp_wmb();
	}
	else
		ret = -EINVAL;
    func_exit();
	return ret;
}
static DEVICE_ATTR(wakeup_enable, S_IWUSR, NULL, wakeup_enable_set);

/**
 * sysf node to check the interrupt status of the sensor, the interrupt
 * handler should perform sysf_notify to allow userland to poll the node.
 */
static ssize_t irq_get(struct device *device,
			struct device_attribute *attribute,
			char* buffer)
{
	struct fpc_data *fpc = dev_get_drvdata(device);
	int irq = gpio_get_value(fpc->irq_gpio);

	return scnprintf(buffer, PAGE_SIZE, "%i\n", irq);
}

/**
 * writing to the irq node will just drop a printk message
 * and return success, used for latency measurement.
 */
static ssize_t irq_ack(struct device *device,
			struct device_attribute *attribute,
			const char *buffer, size_t count)
{
	struct fpc_data *fpc = dev_get_drvdata(device);
    fpsensor_log(DEBUG_LOG, "fpc %s entry.\n", __func__);
	dev_dbg(fpc->dev, "%s\n", __func__);
	return count;
}
static DEVICE_ATTR(irq, S_IRUSR | S_IWUSR, irq_get, irq_ack);

static ssize_t clk_enable_set(struct device *device,
		struct device_attribute *attr, const char *buf, size_t count)
{
    //fpsensor_log(INFO_LOG, "fpc %s entry.\n", __func__);
	return set_clks((*buf == '1')) ? : count;
}
static DEVICE_ATTR(clk_enable, S_IWUSR, NULL, clk_enable_set);

#ifdef FPC_CONFIG_COMPAT
static ssize_t compatible_all_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	
#ifdef PIN_CONTROL
    int i;
#endif
	int irqf;
	int irq_num;
	int rc;
	struct fpc_data *fpc  = dev_get_drvdata(dev);
//	struct spi_device *spidev = fpc->spidev;
    
    fpsensor_log(INFO_LOG, "fpc %s entry, command = %s, compatible_enabled = %d.\n", __func__, buf, fpc->compatible_enabled);
	
	if(!strncmp(buf, "enable", strlen("enable")) && fpc->compatible_enabled != 1 && (!g_broken_flag)){

		fpc->compatible_enabled = 1;
	    fpc->ttw_wl = wakeup_source_create("fpc_ttw_wl");
	    wakeup_source_add(fpc->ttw_wl);
        fpsensor_log(DEBUG_LOG, "%s: %d wakeup_source_add. \n", __func__, __LINE__);
		proc_entry = proc_create(PROC_NAME, 0644, NULL, &proc_file_fpc_ops);
		if (NULL == proc_entry) {
			printk("fpc_fp Couldn't create proc entry!");
			return -ENOMEM;
			} else {
			fp_info_flag = 1;
			printk("fpc_fp Create proc entry success!");
		}
#ifdef SAMSUNG_FP_SUPPORT
        if (fpc_fp_adm)
          {
              fpc_fp_adm = 0;
	          fingerprint_adm_class = class_create(THIS_MODULE,FINGERPRINT_ADM_CLASS_NAME);
	          if (IS_ERR(fingerprint_adm_class))
	          {
		   printk( "fingerprint_adm_class error \n");
		   return PTR_ERR(fingerprint_adm_class);
	          }

	           fingerprint_adm_dev = device_create(fingerprint_adm_class, NULL, 0,
		                                   NULL, FINGERPRINT_ADM_CLASS_NAME);

	           if (sysfs_create_file(&(fingerprint_adm_dev->kobj), &dev_attr_adm.attr))
	           {
		    printk(" fpc fail to creat fingerprint_adm_device \n");
	            }else{
                     printk(" fpc success to creat fingerprint_adm_device \n");
                   }
                   fpc_adm_release =1;
           }
#endif
		rc = fpc_dts_pin_init(fpc);
	    if (rc) {
	        fpsensor_log(ERROR_LOG, "fpc_dts_pin_init fpc init failed\n");
	        return rc;
	    }
	
#ifdef PIN_CONTROL
		fpc->pinctrl_fpc = devm_pinctrl_get(&spidev->dev);
		if (IS_ERR(fpc->pinctrl_fpc)) {
			rc = PTR_ERR(fpc->pinctrl_fpc);
			fpsensor_log(ERROR_LOG, "%s, Cannot find pinctrl_fpc rc = %d.\n", __func__, rc);
			set_clks(false);
			return rc;
		}

		for (i = 0; i < ARRAY_SIZE(fpc->pinctrl_state); i++) {
			const char *n = pctl_names[i];
			struct pinctrl_state *state = pinctrl_lookup_state(fpc->pinctrl_fpc, n);
			if (IS_ERR(state)) {
				fpsensor_log(ERROR_LOG, "%s, cannot find '%s'\n", __func__, n);
				rc = -EINVAL;
				set_clks(false);
				return rc;
			}
			fpsensor_log(INFO_LOG, "%s, found pin control %s\n", __func__, n);
			fpc->pinctrl_state[i] = state;
		}
#endif
		fpc_power_supply(fpc);
		g_fpsensor_clocks_enabled = false;
		set_clks(true);
		(void)hw_reset(fpc);
#ifdef DEBUG_SPI
		rc = check_hwid(spidev);
		if (rc < 0) {
			fpsensor_log(INFO_LOG, "%s: %d get chipid fail. retry... \n", __func__, __LINE__);
            set_clks(false);
#ifdef PIN_CONTROL
            rc = select_pin_ctl(fpc, "fpsensor_fpc_power_low");
            if(rc){
                fpsensor_log(ERROR_LOG, "fpc %s pin ctrl fpsensor_fpc_power_low failed.\n", __func__);
                return rc;
            }
            usleep_range(FPC_POWEROFF_SLEEP_US, FPC_POWEROFF_SLEEP_US + 100);
            
            rc = select_pin_ctl(fpc, "fpsensor_fpc_power_high");
            if(rc){
                fpsensor_log(ERROR_LOG, "fpc %s pin ctrl fpsensor_fpc_power_high failed.\n", __func__);
                return rc;
            }
            usleep_range(FPC_POWEROFF_SLEEP_US, FPC_POWEROFF_SLEEP_US + 100);
#else
         //   gpio_direction_output(fpc->power_ctl_gpio, 0);
         //   usleep_range(FPC_POWEROFF_SLEEP_US, FPC_POWEROFF_SLEEP_US + 100);
          //  gpio_direction_output(fpc->power_ctl_gpio, 1);
         //   usleep_range(FPC_POWEROFF_SLEEP_US, FPC_POWEROFF_SLEEP_US + 100);
#endif
			
			set_clks(true);
			(void)hw_reset(fpc);
			if(check_hwid(spidev))
				fpsensor_log(ERROR_LOG, "%s: %d get chipid fail. \n", __func__, __LINE__);
		}
#endif
		fpc1022_fp_exist = true;

		irq_num = gpio_to_irq(fpc->irq_gpio);

		if (!irq_num) {
			rc = -EINVAL;
			fpsensor_log(ERROR_LOG, "%s, get irq_num error rc = %d.\n", __func__, rc);
			set_clks(false);
			return rc;
		}

		fpsensor_log(INFO_LOG, "%s, Using GPIO#%d as IRQ, Using GPIO#%d as RST, irq_num=%d.\n", __func__, fpc->irq_gpio, fpc->rst_gpio, irq_num);
		fpc->wakeup_enabled = false;

		irqf = IRQF_TRIGGER_HIGH | IRQF_ONESHOT;
		if (of_property_read_bool(dev->of_node, "fpc,enable-wakeup")) {
			irqf |= IRQF_NO_SUSPEND;
			device_init_wakeup(dev, 1);
		}
		rc = devm_request_threaded_irq(dev, irq_num,
			NULL, fpc_irq_handler, irqf,
			dev_name(dev), fpc);
		if (rc) {
			fpsensor_log(ERROR_LOG, "%s, could not request irq %d\n", __func__, irq_num);
			set_clks(false);
			return rc;
		}
		fpsensor_log(INFO_LOG, "%s, requested irq %d\n", __func__, irq_num);

		/* Request that the interrupt should be wakeable */
		enable_irq_wake(irq_num);
		
		hw_reset(fpc);
		
		fpsensor_log(DEBUG_LOG, "fpc %s IRQ_GPIO[%d], RST_GPIO[%d], IRQ after reset is %d\n", __func__, fpc->irq_gpio, fpc->rst_gpio, gpio_get_value(fpc->irq_gpio));
		fpsensor_log(INFO_LOG, "%s: ok\n", __func__);

	}else if(!strncmp(buf, "disable", strlen("disable")) && fpc->compatible_enabled != 0 && (!g_broken_flag)){
		//sysfs_remove_group(&spidev->dev.kobj, &fpc_attribute_group);
		//wakeup_source_trash(&fpc->ttw_wl);
		fpc->compatible_enabled = 0;
		wakeup_source_remove(fpc->ttw_wl);
        fpsensor_log(DEBUG_LOG, "%s: %d wakeup_source_remove. \n", __func__, __LINE__);
		if(fp_info_flag)
			remove_proc_entry(PROC_NAME,NULL);
#ifdef SAMSUNG_FP_SUPPORT
		if(fpc_adm_release)
			{
			device_destroy(fingerprint_adm_class,0);
			class_destroy(fingerprint_adm_class);
                        fpc_adm_release = 0;
			printk("fpc device_remove_file remove! \n");
			}
#endif
		
		if(gpio_is_valid(fpc->irq_gpio)){
			devm_gpio_free(dev, fpc->irq_gpio);
			fpsensor_log(INFO_LOG, "%s, Release IRQ GPIO#%d.\n", __func__, fpc->irq_gpio);
		}
		devm_free_irq(dev, gpio_to_irq(fpc->irq_gpio), fpc);
		fpsensor_log(INFO_LOG, "%s, free IRQ %d.\n", __func__, gpio_to_irq(fpc->irq_gpio));
#ifdef PIN_CONTROL
        rc = select_pin_ctl(fpc, "fpsensor_fpc_power_low");
        if(rc){
            fpsensor_log(ERROR_LOG, "fpc %s pin ctrl fpsensor_fpc_power_low failed.\n", __func__);
            return rc;
        }
#else
      //  gpio_direction_output(fpc->power_ctl_gpio, 0);
#endif
#ifdef USE_REGULATOR
	fpsensor_log(INFO_LOG, "fpc_regulator %s use regulator.\n", __func__);
	set_clks(false);
	rc = vreg_setup(fpc, "vdd_io", false);
	if (rc != 0 ){
		fpsensor_log(ERROR_LOG, "fpc_regulator %s vreg_setup:false failed!\n", __func__);
		return rc;
	}
	fpsensor_log(ERROR_LOG, "fpc_regulator %s vreg_setup:false success!\n", __func__);
#endif
   	//	fpsensor_log(INFO_LOG, "%s, cutoff power fpc->power_ctl_gpio = %d\n", __func__, gpio_get_value(fpc->power_ctl_gpio));
	//	if(gpio_is_valid(fpc->power_ctl_gpio)){
		//	devm_gpio_free(dev, fpc->power_ctl_gpio);
		//	fpsensor_log(INFO_LOG, "%s, Release POWER GPIO#%d.\n", __func__, fpc->power_ctl_gpio);
	//	}
		if(gpio_is_valid(fpc->rst_gpio)){
			devm_gpio_free(dev, fpc->rst_gpio);
			fpsensor_log(INFO_LOG, "%s, Release RST GPIO#%d.\n", __func__, fpc->rst_gpio);
		}
	}
	return count;
}
static DEVICE_ATTR(compatible_all, S_IWUSR, NULL, compatible_all_set);
#endif


static struct attribute *fpc_attributes[] = {
	&dev_attr_hw_reset.attr,
	&dev_attr_wakeup_enable.attr,
	&dev_attr_device_prepare.attr,
	&dev_attr_clk_enable.attr,
	&dev_attr_irq.attr,
#ifdef FPC_CONFIG_COMPAT
	&dev_attr_compatible_all.attr,
#endif
	NULL
};

static const struct attribute_group fpc_attribute_group = {
	.attrs = fpc_attributes,
};

static irqreturn_t fpc_irq_handler(int irq, void *handle)
{
	struct fpc_data *fpc = handle;
	struct device *dev = fpc->dev;
	static int current_level = 0; // We assume low level from start
	current_level = !current_level;

	if (current_level) {
		dev_dbg(dev, "Reconfigure irq to trigger in low level\n");
		irq_set_irq_type(irq, IRQF_TRIGGER_LOW);
	} else {
		dev_dbg(dev, "Reconfigure irq to trigger in high level\n");
		irq_set_irq_type(irq, IRQF_TRIGGER_HIGH);
	}

	/* Make sure 'wakeup_enabled' is updated before using it
	** since this is interrupt context (other thread...) */
	smp_rmb();
	if (fpc->wakeup_enabled) {
		__pm_wakeup_event(fpc->ttw_wl, FPC_TTW_HOLD_TIME);
	}

	sysfs_notify(&dev->kobj, NULL, dev_attr_irq.attr.name);

	return IRQ_HANDLED;
}

static int mtk6797_probe(struct spi_device *spidev)
{
	struct device *dev = &spidev->dev;
	struct device_node *fpc_node;
	struct fpc_data *fpc;
	int rc = 0;
	reset_num = 0;
	fpc_fp_adm = 1;
#ifndef FPC_CONFIG_COMPAT
#ifdef PIN_CONTROL
    size_t i;
#endif 
	int irqf = 0;
	int irq_num = 0;
#endif
	fpsensor_log(INFO_LOG, "fpc %s entry.\n", __func__);

	fpc_node = spidev->dev.of_node;
	spidev->dev.of_node = of_find_compatible_node(NULL, NULL, "mediatek,fpsensor");
	if (!fpc_node) {
		fpsensor_log(ERROR_LOG, "%s, no of node found\n", __func__);
		rc = -EINVAL;
		return rc;
	}

	fpc = devm_kzalloc(dev, sizeof(*fpc), GFP_KERNEL);
	if (!fpc) {
		fpsensor_log(ERROR_LOG, "%s, failed to allocate memory for struct fpc_data\n", __func__);
		rc = -ENOMEM;
		return rc;
	}

	fpc->dev = dev;
	dev_set_drvdata(dev, fpc);
	fpc->spidev = spidev;
	fpc->spidev->irq = 0; /*SPI_MODE_0*/
	fpc->spidev->mode = SPI_MODE_0;
	fpc->spidev->bits_per_word = 8;
	fpc->spidev->max_speed_hz = 1 * 1000 * 1000;
	g_fpsensor_spidev = spidev;

#ifndef FPC_CONFIG_COMPAT
	fpsensor_log(INFO_LOG, "fpc %s entry. FPC_CONFIG_COMPAT\n", __func__);
	rc = fpc_dts_pin_init(fpc);
	if (rc) {
	    fpsensor_log(ERROR_LOG, "fpc_dts_pin_init fpc init failed\n");
	    return rc;
	}

#ifdef PIN_CONTROL
	fpsensor_log(INFO_LOG, "fpc %s entry. PIN_CONTROL\n", __func__);
	fpc->pinctrl_fpc = devm_pinctrl_get(&spidev->dev);
	if (IS_ERR(fpc->pinctrl_fpc)) {
		rc = PTR_ERR(fpc->pinctrl_fpc);
		fpsensor_log(ERROR_LOG, "%s, Cannot find pinctrl_fpc rc = %d.\n", __func__, rc);
		set_clks(false);
		return rc;
	}

	for (i = 0; i < ARRAY_SIZE(fpc->pinctrl_state); i++) {
		const char *n = pctl_names[i];
		struct pinctrl_state *state = pinctrl_lookup_state(fpc->pinctrl_fpc, n);
		if (IS_ERR(state)) {
			fpsensor_log(ERROR_LOG, "%s, cannot find '%s'\n", __func__, n);
			rc = -EINVAL;
			set_clks(false);
			return rc;
		}
		fpsensor_log(INFO_LOG, "%s found pin control %s\n", __func__, n);
		fpc->pinctrl_state[i] = state;
	}
#endif
	fpc_power_supply(fpc);
	g_fpsensor_clocks_enabled = false;
	set_clks(true);
	fpsensor_log(INFO_LOG, "fpc %s entry. hw_reset B\n", __func__);
	(void)hw_reset(fpc);
	fpsensor_log(INFO_LOG, "fpc %s entry. hw_reset E\n", __func__);
#ifdef DEBUG_SPI
	rc = check_hwid(spidev);
	if (rc < 0) {
			fpsensor_log(ERROR_LOG, "%s: %d get chipid fail. now exit\n",
				  __func__, __LINE__);
#ifdef PIN_CONTROL
            rc = select_pin_ctl(fpc, "fpsensor_fpc_power_low");
            if(rc){
                fpsensor_log(ERROR_LOG, "fpc %s pin ctrl fpsensor_fpc_power_low failed.\n", __func__);
                return rc;
            }
#else
       //     gpio_direction_output(fpc->power_ctl_gpio, 0);
#endif
			gpio_free(fpc->rst_gpio);
			set_clks(false);
			fpc1022_fp_exist = false;
			spidev->dev.of_node = fpc_node;
			return -EAGAIN;
	}
#endif

	fpc1022_fp_exist = true;

	irq_num = gpio_to_irq(fpc->irq_gpio);

	if (!irq_num) {
		rc = -EINVAL;
		fpsensor_log(ERROR_LOG, "%s get irq_num error rc = %d.\n", __func__, rc);
		set_clks(false);
		return rc;
	}

	fpsensor_log(INFO_LOG, "%s, Using GPIO#%d as IRQ, Using GPIO#%d as RST. \n", __func__, fpc->irq_gpio, fpc->rst_gpio);
	fpc->wakeup_enabled = false;

	irqf = IRQF_TRIGGER_HIGH | IRQF_ONESHOT;
	if (of_property_read_bool(dev->of_node, "fpc,enable-wakeup")) {
		irqf |= IRQF_NO_SUSPEND;
		device_init_wakeup(dev, 1);
	}
	rc = devm_request_threaded_irq(dev, irq_num,
		NULL, fpc_irq_handler, irqf,
		dev_name(dev), fpc);
	if (rc) {
		fpsensor_log(ERROR_LOG, "%s, could not request irq %d\n", __func__, irq_num);
		set_clks(false);
		return rc;
	}
	fpsensor_log(INFO_LOG, "%s, requested irq %d\n", __func__, irq_num);

	/* Request that the interrupt should be wakeable */
	enable_irq_wake(irq_num);
	//wakeup_source_init(&fpc->ttw_wl, "fpc_ttw_wl");
	
	fpc->ttw_wl = wakeup_source_create("fpc_ttw_wl");
	wakeup_source_add(fpc->ttw_wl);

	rc = sysfs_create_group(&dev->kobj, &fpc_attribute_group);
	if (rc) {
		fpsensor_log(ERROR_LOG, "%s, could not create sysfs\n", __func__);
		set_clks(false);
		return rc;
	}

	(void)hw_reset(fpc);
	fpsensor_log(INFO_LOG, "%s: ok\n", __func__);
    func_exit();
	return rc;
#else
	//fpc->ttw_wl = wakeup_source_create("fpc_ttw_wl");
	//wakeup_source_add(fpc->ttw_wl);
	rc = sysfs_create_group(&dev->kobj, &fpc_attribute_group);
	if (rc) {
		fpsensor_log(ERROR_LOG, "%s, could not create sysfs\n", __func__);
		return rc;
	}
	fpsensor_log(INFO_LOG, "%s: ok\n", __func__);
    func_exit();
	return rc;
#endif 

}

static int mtk6797_remove(struct spi_device *spidev)
{
	struct  fpc_data *fpc = dev_get_drvdata(&spidev->dev);
    fpsensor_log(INFO_LOG, "fpc %s entry.\n", __func__);
	sysfs_remove_group(&spidev->dev.kobj, &fpc_attribute_group);
	//wakeup_source_trash(&fpc->ttw_wl);
	remove_proc_entry(PROC_NAME,NULL);
#ifdef SAMSUNG_FP_SUPPORT	
	    if(fpc_adm_release)
        {
        device_destroy(fingerprint_adm_class,0);
        class_destroy(fingerprint_adm_class);
        fpc_adm_release = 0;
        printk("fpc device_remove_file remove! \n");
        }
#endif
	wakeup_source_remove(fpc->ttw_wl);
	
	return 0;
}

static struct of_device_id mt6797_of_match[] = {
	{ .compatible = "mediatek,fpsensor", },
	{}
};
MODULE_DEVICE_TABLE(of, mt6797_of_match);

static struct spi_driver mtk6797_driver = {
	.driver = {
		.name	= "fpsensor",
		.bus = &spi_bus_type,
		.owner	= THIS_MODULE,
		.of_match_table = mt6797_of_match,
	},
	.probe	= mtk6797_probe,
	.remove	= mtk6797_remove
};

static int __init fpc_sensor_init(void)
{
	int status;
    func_entry();
	status = spi_register_driver(&mtk6797_driver);
	if (status < 0) {
		fpsensor_log(ERROR_LOG, "%s, fpc_sensor_init failed.\n", __func__);
	}

#ifdef FPC_DRIVER_FOR_ISEE
	memcpy(&uuid_fp, &uuid_ta_fpc, sizeof(struct TEEC_UUID));
#endif

	return status;
}
module_init(fpc_sensor_init);

static void __exit fpc_sensor_exit(void)
{
    func_exit();
	spi_unregister_driver(&mtk6797_driver);
}
module_exit(fpc_sensor_exit);

MODULE_LICENSE("GPL");
