/*
 *  pwm_device_driver.c
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>         // Header to support the kernel Driver Model
#include <linux/kernel.h>         // Contains types, macros, functions for the kernel
#include <linux/fs.h>             // Header for the Linux file system support
#include <asm/uaccess.h>          // Required for the copy to user function
#include <linux/gpio.h>     // library for the gpio of, in this case, the raspberry pi
#include <linux/hrtimer.h>
#include <linux/ktime.h>

MODULE_LICENSE("Dual BSD/GPL");
#define BUF_LEN 256 //max length oof msg

#define PWM_MOTOR_LABEL "GPIO18_PWM_MOTOR"
#define PWM_MOTOR_GPIO 18
#define PWM_AXIS_LABEL "GPIO17_PWM_AXIS"
#define PWM_AXIS_GPIO 17

#define DEVICE_CLASS_NAME "pwm_cl"
#define DEVICE_DRIVER_NAME "pwm_drv"
#define DEVICE_DRIVER_MAJOR 0   // dynamic allocation

#define MS_TO_NS(x) (x * 1E6L)
#define US_TO_NS(x) (x * 1E3L)
 
static unsigned long count = 0;

static struct hrtimer hr_timer0;
 
enum hrtimer_restart hr_timer0_callback( struct hrtimer *timer )
{
    ktime_t ktime0;
    ktime0 = ktime_set( 0, US_TO_NS(16667L) );
    hrtimer_start( &hr_timer0, ktime0, HRTIMER_MODE_REL );
    count++;
    // if (count == 1)
    if (count%100 <= 1)
    {
        printk( "hrtimer0_callback called (%ld), %ld.\n", jiffies, count%2 );
    }

    gpio_set_value(PWM_MOTOR_GPIO, (count%2));

    return HRTIMER_RESTART;
}

int init_pwm_gpio(void);
void uninit_pwm_gpio(void);
int init_pwm_timers(void);
void uninit_pwm_timers(void);

static int cdd_mj;
static char gpio_select = 'U';
static struct class *cddcharClass = NULL;
static struct device *cddcharDevice = NULL;

static char msg[BUF_LEN];   /* The msg the device will give when asked */
static char *msg_Ptr;


static ssize_t cdd_open(struct inode *i, struct file *filp);
static ssize_t cdd_release(struct inode *i, struct file *filp);
static ssize_t cdd_read(struct file *filp, char *buff, size_t count, loff_t *offp);
static ssize_t cdd_write(struct file *filp, const char *buff, size_t count, loff_t *offp);

static struct file_operations cdd_fops = {
    .open = cdd_open,
    .read = cdd_read,
    .release = cdd_release,
    .write = cdd_write
};

int init_chard(void);
int exit_chard(void);

static int hello_init(void)
{
    int retv; 
    printk(KERN_ALERT "\n\n\nHello, minor\n");
    retv = init_chard();
    retv |= init_pwm_gpio();
    retv |= init_pwm_timers();
    return retv;
}

static void hello_exit(void)
{
    int retv;
    printk(KERN_ALERT "Goodbye, embedded minor\n\n\n");
    retv=exit_chard();
    uninit_pwm_gpio();
    uninit_pwm_timers();
}

module_init(hello_init);
module_exit(hello_exit);

int init_chard(void) 
{
    // dynamically allocate major number
    cdd_mj = register_chrdev(DEVICE_DRIVER_MAJOR, DEVICE_DRIVER_NAME, &cdd_fops);   // allocate char driver dynamically and returns major number;
    if (cdd_mj < 0)
    {
        printk(KERN_ALERT "Failed to register a major number\n");
        return cdd_mj;
    }
    printk(KERN_ALERT "Succesfully registered major number %d\n", cdd_mj);

    // register char device class
    cddcharClass = class_create(THIS_MODULE, DEVICE_CLASS_NAME);
    if (IS_ERR(cddcharClass)){                  // Check for error and clean up if there is
        unregister_chrdev(cdd_mj, DEVICE_CLASS_NAME);
        printk(KERN_ALERT "Failed to register device class\n");
        return PTR_ERR(cddcharClass);           // Correct way to return an error on a pointer
    }
    printk(KERN_INFO "Device class with name \"%s\" registered correctly\n", DEVICE_CLASS_NAME);

    // Register the device driver
    cddcharDevice = device_create(cddcharClass, NULL, MKDEV(cdd_mj, 0), NULL, DEVICE_DRIVER_NAME);
    if (IS_ERR(cddcharDevice)){                 // Clean up if there is an error
        class_destroy(cddcharClass);            // Repeated code but the alternative is goto statements
        unregister_chrdev(cdd_mj, DEVICE_DRIVER_NAME);
        printk(KERN_ALERT "Failed to create the device\n");
        return PTR_ERR(cddcharDevice);
    }
    printk(KERN_INFO "Device driver with name \"%s\" created correctly\n", DEVICE_DRIVER_NAME); // Made it! device was initialized
    return 0;
}


int exit_chard(void) 
{
    device_destroy(cddcharClass, MKDEV(cdd_mj, 0)); // remove the device
    class_unregister(cddcharClass);                 // unregister the device class
    class_destroy(cddcharClass);                    // remove the device class
    unregister_chrdev(cdd_mj, DEVICE_DRIVER_NAME);  // unregister the major number
    return 0;
}

int init_pwm_gpio(void)
{
    int error = 0;
    // error = gpio_request(ld1_gpio, ld1_label);
    error = gpio_request(PWM_MOTOR_GPIO, PWM_MOTOR_LABEL);
    if (error != 0)
    {
        gpio_free(PWM_MOTOR_GPIO);
    }
    // error = gpio_request(ld2_gpio, ld2_label);
    error = gpio_request(PWM_AXIS_GPIO, PWM_AXIS_LABEL);
    if (error != 0)
    {
        gpio_free(PWM_AXIS_GPIO);
    }
    error = gpio_direction_output(PWM_MOTOR_GPIO, 0);
    error = gpio_direction_output(PWM_AXIS_GPIO, 0);
    return error;
}

void uninit_pwm_gpio(void)
{
    gpio_free(PWM_MOTOR_GPIO);
    gpio_free(PWM_AXIS_GPIO);
}


int init_pwm_timers(void)
{
    ktime_t ktime0;
    unsigned long delay_in_ms = 1L;

    printk("HR Timer module installing\n");

    ktime0 = ktime_set( 0, MS_TO_NS(delay_in_ms) );

    hrtimer_init( &hr_timer0, CLOCK_MONOTONIC, HRTIMER_MODE_REL );

    hr_timer0.function = &hr_timer0_callback;

    printk( "Starting timer to fire in %ldms (%ld)\n", delay_in_ms, jiffies );

    hrtimer_start( &hr_timer0, ktime0, HRTIMER_MODE_REL );

    return 0;
}

void uninit_pwm_timers(void)
{
    int ret;
    ret = hrtimer_cancel( &hr_timer0 );
    if (ret) printk("The timer was still in use...\n");

    printk("HR Timer module uninstalling\n");
}


static ssize_t cdd_open(struct inode *i, struct file *filp)
{
    // not much happens
    printk(KERN_INFO "Embedded: Device open()\n");
    msg_Ptr = msg;
    return 0;
}

static ssize_t cdd_release(struct inode *i, struct file *filp)
{
    // not much happens
    printk(KERN_INFO "Embedded: Device close()\n");
    return 0;
}

// static ssize_t cdd_read(struct file *filp, char *buff, size_t count, loff_t *offp)
static ssize_t cdd_read(struct file *filp,   /* see include/linux/fs.h   */
               char *buffer,    /* buffer to fill with data */
               size_t length,   /* length of the buffer     */
               loff_t * offset)
{

    /*
     * Number of bytes actually written to the buffer 
     */
    int bytes_read = 0;

    if (gpio_select == 'D')
    {
        // msg[0] = gpio_get_value(S1_GPIO) + '0';
        // msg[1] = '\n';   // new line
    }
    else
    {
        msg[0] = '\0';  // empty
        // msg[1] = '\0';  // empty
    }

    /*
     * If we're at the end of the message, 
     * return 0 signifying end of file 
     */
    if (*msg_Ptr == 0)
    {
        printk(KERN_INFO "Embedded: Device read() done!\n");
        return 0;
    }

    /* 
     * Actually put the data into the buffer 
     */
    while (length && *msg_Ptr) {

        /* 
         * The buffer is in the user data segment, not the kernel 
         * segment so "*" assignment won't work.  We have to use 
         * put_user which copies data from the kernel data segment to
         * the user data segment. 
         */

        put_user(*(msg_Ptr++), buffer++);
        length--;
        bytes_read++;
    }

    printk(KERN_INFO "Embedded: Device read() Sent %d characters to the user\n", bytes_read);

    /* 
     * Most read functions return the number of bytes put into the buffer
     */
    return bytes_read;

}

static ssize_t cdd_write(struct file *filp, const char *buff, size_t count, loff_t *offp)
{
    int i = 0;
    // reading
    // printk(KERN_INFO "Embedded: Device write() %s %d\n", buff, count);

    for(i = 0; i < count; i++)
    // while (i < count)
    {
        // printk(KERN_INFO "Embedded: Device write() %c\n", buff[i]);
        // switch(buff[i])
        // {
        //     case 'C':
        //     case 'D':
        //         gpio_select = buff[i];
        //         break;

        //     case '0':
        //         if (gpio_select == 'C')
        //         {
        //             gpio_set_value(LD1_GPIO, 0);
        //             gpio_set_value(LD2_GPIO, 0);
        //         }
        //         break;

        //     case '1':
        //         if (gpio_select == 'C')
        //         {
        //             gpio_set_value(LD1_GPIO, 1);
        //         }
        //         break;


        //     case '2':
        //         if (gpio_select == 'C')
        //         {
        //             gpio_set_value(LD2_GPIO, 1);
        //         }
        //         break;
                
        //     // if (gpio_select == 'C')
        //     // {
        //     //     case '0':
        //     //         gpio_set_value(LD1_GPIO, 0);
        //     //         gpio_set_value(LD2_GPIO, 0);
        //     //         break;

        //     //     case '1':
        //     //         gpio_set_value(LD1_GPIO, 1);
        //     //         break;

        //     //     case '2':
        //     //         gpio_set_value(LD2_GPIO, 1);
        //     //         break;
        //     // }
        // }
        // i++;
    }
    return count;
}

/*
 *  pwm_device_driver.c - EOF
 */