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

//MODULE_LICENSE("Dual BSD/GPL");
MODULE_LICENSE("GPL");
#define BUF_LEN 256 //max length oof msg

#define PWM_MOTOR_LABEL "GPIO18_PWM_MOTOR"
#define PWM_MOTOR_GPIO 19
#define PWM_AXIS_LABEL "GPIO17_PWM_AXIS"
#define PWM_AXIS_GPIO 26

#define DEVICE_CLASS_NAME "pwm_cl"
#define DEVICE_DRIVER_NAME "pwm_drv"
#define DEVICE_DRIVER_MAJOR 0   // dynamic allocation

#define MS_TO_NS(x)     (x * 1E6L)
#define US_TO_NS(x)     (x * 1E3L)
#define SYNC_PWM_NS     16666667    // 60 Hertz
#define SYNC_HIGH_NS     1520000    // 1.52 ms
#define SYNC_LOW_NS     (SYNC_PWM_NS - SYNC_HIGH_NS)
#define PWM_MOTOR_DEFAULT_VALUE 1520
#define PWM_AXIS_DEFAULT_VALUE  1380

static struct hrtimer hr_timer0;    // PWM timer

static unsigned int motor_uptime_us, axis_uptime_us; // uptime value of the pwm's in microseconds
static unsigned int new_motor_uptime_us, new_axis_uptime_us; // uptime value of the pwm's in microseconds
static int sync_counter = 0;

enum timer_states 
{
    state_idle,
    state_sync,
    state_run,
    state_motor_pwm,
    state_axis_pwm,
    state_rem_motor_pwm,
    state_rem_axis_pwm,
    state_rem_down_time
};

enum timer_states timer_state, next_timer_state;
 
enum hrtimer_restart hr_timer0_callback( struct hrtimer *timer )
{
    ktime_t ktime0;
    timer_state = next_timer_state;
     

    if (timer_state == state_run)
    {
        motor_uptime_us = new_motor_uptime_us;
        axis_uptime_us = new_axis_uptime_us;
        if (motor_uptime_us <= axis_uptime_us)
        {
            next_timer_state = state_motor_pwm;
        } else {
            next_timer_state = state_axis_pwm;
        }
        // printk(KERN_INFO "%u %u\n", motor_uptime_us, axis_uptime_us);
        gpio_set_value(PWM_MOTOR_GPIO, 1);
        gpio_set_value(PWM_AXIS_GPIO, 1);
        timer_state = next_timer_state;
    }


    if (timer_state == state_idle)
    {
        // do nothing
        ktime0 = ktime_set( 0, (SYNC_LOW_NS * 1L) );
        gpio_set_value(PWM_MOTOR_GPIO, 0);
        gpio_set_value(PWM_AXIS_GPIO, 0);
    } 
    else if (timer_state == state_sync)
    {
        new_motor_uptime_us = PWM_MOTOR_DEFAULT_VALUE;
        new_axis_uptime_us = PWM_AXIS_DEFAULT_VALUE;
        if (sync_counter%2 == 0)
        {
            ktime0 = ktime_set( 0, (SYNC_HIGH_NS * 1L) );
            gpio_set_value(PWM_MOTOR_GPIO, 1);
        }
        else if (sync_counter%2 == 1)
        {
            ktime0 = ktime_set( 0, (SYNC_LOW_NS * 1L) );
            gpio_set_value(PWM_MOTOR_GPIO, 0);
        }

        if (sync_counter >= 20)
        {
            sync_counter = 0;
            next_timer_state = state_run;
        }
        sync_counter++;
    }
    else if (timer_state == state_motor_pwm)
    {
        ktime0 = ktime_set( 0, (motor_uptime_us * 1000L) );
        next_timer_state = state_rem_axis_pwm;        
    }
    else if (timer_state == state_axis_pwm)
    {
        // printk(KERN_ALERT "Debug\n");
        ktime0 = ktime_set( 0, (axis_uptime_us * 1000L) );  
        next_timer_state = state_rem_motor_pwm;
        // printk(KERN_ALERT "Debug After\n");
    }
    else if (timer_state == state_rem_motor_pwm)
    {
        ktime0 = ktime_set( 0, (motor_uptime_us - axis_uptime_us) * 1000L );
        gpio_set_value(PWM_AXIS_GPIO, 0); 
        next_timer_state = state_rem_down_time;
        // printk(KERN_INFO "rem_ns: %d\n", (motor_uptime_us - axis_uptime_us) * 1000L);
    }
    else if (timer_state == state_rem_axis_pwm)
    {
        ktime0 = ktime_set( 0, (axis_uptime_us - motor_uptime_us) * 1000L );
        gpio_set_value(PWM_MOTOR_GPIO, 0);
        next_timer_state = state_rem_down_time;
    }
    else if (timer_state == state_rem_down_time)
    {
        unsigned int x;
        if (motor_uptime_us < axis_uptime_us)
        {
            x = SYNC_PWM_NS - (axis_uptime_us * 1000);
        } else {
            x = SYNC_PWM_NS - (motor_uptime_us * 1000);
        }
        ktime0 = ktime_set( 0, (x * 1L) );    
        // printk(KERN_INFO "downtime_ns: %d\n", x);
        gpio_set_value(PWM_MOTOR_GPIO, 0);
        gpio_set_value(PWM_AXIS_GPIO, 0);
        next_timer_state = state_run;
    }
    else
    {
        next_timer_state = state_idle;
        gpio_set_value(PWM_MOTOR_GPIO, 0);
        gpio_set_value(PWM_AXIS_GPIO, 0);
        ktime0 = ktime_set( 0, SYNC_PWM_NS * 1L);    
        printk(KERN_INFO "Not recoqnize state. reset\n");
    }
    
    hrtimer_start( &hr_timer0, ktime0, HRTIMER_MODE_REL );

    return HRTIMER_RESTART;
}

int init_pwm_gpio(void);
void uninit_pwm_gpio(void);
int init_pwm_timers(void);
void uninit_pwm_timers(void);

static int cdd_mj;
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
    motor_uptime_us = 1520;     // micros
    axis_uptime_us  = 1380;     // micros
    new_motor_uptime_us = motor_uptime_us;
    new_axis_uptime_us = axis_uptime_us;

    unsigned long delay_in_ms;
    delay_in_ms = 1L;

    printk("HR Timer module installing\n");

    // timer 0
    ktime0 = ktime_set( 0, (SYNC_PWM_NS * 1L) );
    hrtimer_init( &hr_timer0, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
    hr_timer0.function = &hr_timer0_callback;
    timer_state = state_idle;
    next_timer_state = timer_state;

    printk( "Starting timer to fire in %ldms (%ld)\n", delay_in_ms, jiffies );

    hrtimer_start( &hr_timer0, ktime0, HRTIMER_MODE_REL );

    return 0;
}

void uninit_pwm_timers(void)
{
    int ret;
    ret = hrtimer_cancel( &hr_timer0 );

    if (ret) printk("The timers were still in use...\n");

    printk("HR Timer module uninstalling\n");
}


static ssize_t cdd_open(struct inode *i, struct file *filp)
{
    // not much happens
    // printk(KERN_INFO "Embedded: Device open()\n");
    msg_Ptr = msg;
    return 0;
}

static ssize_t cdd_release(struct inode *i, struct file *filp)
{
    // not much happens
    // printk(KERN_INFO "Embedded: Device close()\n");
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

    char buffer_to_send[15] = "F60MUUUUAUUUU"; // UUUU's will be set later
    buffer_to_send[4] = (motor_uptime_us / 1000) + '0';   // motor speed;
    buffer_to_send[5] = ((motor_uptime_us / 100) % 10) + '0';
    buffer_to_send[6] = ((motor_uptime_us / 10) % 10) + '0';
    buffer_to_send[7] = (motor_uptime_us % 10) + '0';

    buffer_to_send[9] = (axis_uptime_us / 1000) + '0';   // motor speed;
    buffer_to_send[10] = ((axis_uptime_us / 100) % 10) + '0';
    buffer_to_send[11] = ((axis_uptime_us / 10) % 10) + '0';
    buffer_to_send[12] = (axis_uptime_us % 10) + '0';

    buffer_to_send[13] = '\n';
    buffer_to_send[14] = '\0';

    int i;
    for (i = 0; i < 15; i++)
    {
        msg[i] = buffer_to_send[i];
    }

    /*
     * If we're at the end of the message, 
     * return 0 signifying end of file 
     */
    if (*msg_Ptr == 0)
    {
        // printk(KERN_INFO "Embedded: Device read() done!\n");
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

    // printk(KERN_INFO "Embedded: Device read() Sent %d characters to the user\n", bytes_read);

    /* 
     * Most read functions return the number of bytes put into the buffer
     */
    return bytes_read;

}

static ssize_t cdd_write(struct file *filp, const char *buff, size_t count, loff_t *offp)
{
    int i = 0;
    char buffer[10];
    // reading
    // printk(KERN_INFO "Embedded: Device write() %s %d\n", buff, count);


    for(i = 0; i < count; i++)
    {
        // fetch the buffer
        buffer[i] = buff[i];
        
    }
    // printk(KERN_INFO "Embedded: Device write() %s\n", *buffer);

    switch(buffer[0])
    {
        case 'S': 
            next_timer_state = state_sync;
            printk(KERN_INFO "Embedded: Device going to start/sync\n");
            // printk(KERN_INFO "State: %d\n", timer_state);
            break;

        case 'C':
            next_timer_state = state_idle;
            printk(KERN_INFO "Embedded: Device stopping\n");
            // printk(KERN_INFO "State: %d\n", timer_state);
            break;

        case 'M':
            new_motor_uptime_us = (unsigned int) ( 1000*(buffer[1] - '0') + 100*(buffer[2] - '0') + 10*(buffer[3] - '0') + (buffer[4] - '0') );
            if (!(new_motor_uptime_us >= 1000 && new_motor_uptime_us <= 2000))
            {
                new_motor_uptime_us = PWM_MOTOR_DEFAULT_VALUE;
            }
            printk(KERN_INFO "Embedded: Device new Motor value %u\n", new_motor_uptime_us);
            break;

        case 'A':
            new_axis_uptime_us = (unsigned int) ( 1000*(buffer[1] - '0') + 100*(buffer[2] - '0') + 10*(buffer[3] - '0') + (buffer[4] - '0') );
            if (!(new_axis_uptime_us >= 1000 && new_axis_uptime_us <= 2000))
            {
                new_axis_uptime_us = PWM_MOTOR_DEFAULT_VALUE;
            }
            printk(KERN_INFO "Embedded: Device new Axis value %u\n", new_axis_uptime_us);
            break;
    }

    return count;
}

/*
 *  pwm_device_driver.c - EOF
 */
