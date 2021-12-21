#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/net.h>
#include <net/sock.h>
#include <linux/socket.h>
#include <linux/kthread.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/completion.h>
#include <linux/spinlock.h>
#include <linux/jiffies.h>
#include <linux/mutex.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Connor Claypool");
MODULE_DESCRIPTION("A networked GPIO driver");
MODULE_VERSION("0.1");

#define PORT 12345

#define MSG_LEN 2
#define MSG_PIN_IDX 0
#define MSG_VALUE_IDX 1

#define GPIO_OUTPUT_START 4
#define GPIO_OUTPUT_END 8
#define NUM_GPIO_OUTPUT 4
#define GPIO_INPUT_RISING_START 8
#define GPIO_INPUT_RISING_END 12
#define GPIO_INPUT_FALLING_START 16
#define GPIO_INPUT_FALLING_END 20
#define NUM_GPIO_INPUT 4
#define NUM_GPIO_PINS (NUM_GPIO_INPUT * 2 + NUM_GPIO_OUTPUT)

#define GPIO_LABEL_LEN 11
#define GPIO_OUTPUT_OFFSET 0
#define GPIO_INPUT_RISING_OFFSET 6
#define GPIO_INPUT_FALLING_OFFSET 12

#define GPIO_EVENT_QUEUE_SIZE 128

struct GPIOEvent {
    uint8_t pin;
    uint8_t value;
};
typedef struct GPIOEvent GPIOEvent_t;

struct GPIOInfo {
    // bit field for pin values
    uint8_t pin_values;
    // bit field for whether values have changed
    uint8_t values_changed;
};
typedef struct GPIOInfo GPIOInfo_t;

struct GPIOEventQueue {
    // event
    GPIOEvent_t ev;
    // list head
    struct list_head list;
};
typedef struct GPIOEventQueue GPIOEventQueue_t;

// socket for sending and receiving messages
struct socket *conn = NULL;
// kthread to connect to server
struct task_struct *connect_kthread = NULL;
// kthread to receive network messages
struct task_struct *receive_messages_kthread = NULL;
// kthread to send network messages
struct task_struct *send_messages_kthread = NULL;
// pointers to allocated GPIO labels
char *gpio_labels[NUM_GPIO_PINS] = { NULL };
// record of held GPIO pins for cleanup
int allocated_gpio_pins[NUM_GPIO_PINS] = { 0 };
// record of allocated IRQs
int allocated_irqs[NUM_GPIO_PINS] = { 0 };
// last trigger time for debouncing GPIO interrupts
unsigned long last_interrupts[NUM_GPIO_PINS] = { 0 };
// completion to signal that there is an item in the event queue
DECLARE_COMPLETION(gpio_event_added);
// whether to keep re-initializing the completion
bool gpio_event_added_active = true;
// mutex to protect controlling reinitialization of completion
DEFINE_MUTEX(gpio_event_added_reinit_lock);
// ring buffer queue for GPIO events
GPIOEvent_t gpio_events[GPIO_EVENT_QUEUE_SIZE];
// read index
int gpio_event_read = 0;
// write index
int gpio_event_write = 0;
// items in buffer
int gpio_events_items = 0;
// spinlock protecting ring buffer queue
DEFINE_SPINLOCK(gpio_events_lock);

// check if queue is empty
bool gpio_events_empty(void)
{
    return (gpio_events_items == 0);
}

// check if queue is full
bool gpio_events_full(void)
{
    return (gpio_events_items == GPIO_EVENT_QUEUE_SIZE);
}

// push a GPIO event onto the queue
int gpio_events_push(GPIOEvent_t ev)
{
    if (gpio_events_full()) {
        return -1;
    }
    // pr_info("Writing at: %d\n", gpio_event_write);
    gpio_events[gpio_event_write] = ev;
    gpio_event_write += 1;
    if (gpio_event_write >= GPIO_EVENT_QUEUE_SIZE) {
        gpio_event_write = 0;
    }
    // pr_info("New write value: %d\n", gpio_event_write);
    gpio_events_items += 1;
    // pr_info("Write: queue size %d\n", gpio_events_items);
    return 0;
}

// pop a GPIO event off the queue. Assumes the queue is not empty!
GPIOEvent_t gpio_events_pop(void)
{
    GPIOEvent_t ev;
    // pr_info("Reading at: %d\n", gpio_event_read);
    ev = gpio_events[gpio_event_read];
    gpio_event_read += 1;
    if (gpio_event_read >= GPIO_EVENT_QUEUE_SIZE) {
        gpio_event_read = 0;
    }
    // pr_info("New read value: %d\n", gpio_event_read);
    gpio_events_items -= 1;
    // pr_info("Read: queue size %d\n", gpio_events_items);
    return ev;
}

// construct a 32-bit IPv4 address from four octets
uint32_t format_ip_address(uint8_t *octets)
{
    uint32_t address = 0;
    int i;
    for (i = 0; i < 4; i++) {
        address |= (octets[i] << ((3 - i) * 8));
    }
    return address;
}

// initialize socket
int init_socket(void)
{
    int ret;

    ret = sock_create(PF_INET, SOCK_STREAM, IPPROTO_TCP, &conn);
    if (ret < 0) {
        pr_info("Error creating socket: %d\n", ret);
    }

    return ret;
}

// calculate delay for connection attempts
int calculate_delay(int iteration)
{
    if (iteration < 100) {
        return 100;
    } else if (iteration < 1000) {
        return 1000;
    }
    return 2500;
}

// convert GPIO to sequence number
int gpio_to_seq(int gpio_number)
{
    if (gpio_number >= GPIO_OUTPUT_START && gpio_number < GPIO_OUTPUT_END)
    {
        return GPIO_OUTPUT_OFFSET + (gpio_number - GPIO_OUTPUT_START);
    }
    else if (gpio_number >= GPIO_INPUT_RISING_START && gpio_number < GPIO_INPUT_RISING_END)
    {
        return GPIO_INPUT_RISING_OFFSET + (gpio_number - GPIO_INPUT_RISING_START);
    }
    else if (gpio_number >= GPIO_INPUT_FALLING_START && gpio_number < GPIO_INPUT_FALLING_END)
    {
        return GPIO_INPUT_FALLING_OFFSET + (gpio_number - GPIO_INPUT_FALLING_START);
    }
    return -1;
}

// GPIO interrupt handler
static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
    int err;
    unsigned long diff;
    int i;
    bool rising;
    bool found;
    int input_number;
    unsigned long flags;
    GPIOEvent_t ev;

    // find GPIO number and whether this is rising or falling edge
    found = false;
    for (i = 0; !found && i < NUM_GPIO_INPUT; i++) {
        if (gpio_to_irq(GPIO_INPUT_RISING_START + i) == irq) {
            rising = true;
            input_number = i;
            found = true;
        }
    }
    for (i = 0; !found && i < NUM_GPIO_INPUT; i++) {
        if (gpio_to_irq(GPIO_INPUT_FALLING_START + i) == irq) {
            rising = false;
            input_number = i;
            found = true;
        }
    }

    if (!found) {
        pr_info("Error: unknown GPIO interrupt!\n");
        //spin_unlock_irqrestore(&gpio_input_info_lock, flags);
        return IRQ_HANDLED;
    }

    //pr_info("Before debounce GPIO Interrupt: %d. Rising edge: %d\n", gpio_number, rising);

    // debounce
    diff = jiffies - last_interrupts[input_number];
    if (diff < 5) {
        // spin_unlock_irqrestore(&gpio_input_info_lock, flags);
        return IRQ_HANDLED;
    }
    last_interrupts[input_number] = jiffies;


    // acquire lock to add event to queue
    spin_lock_irqsave(&gpio_events_lock, flags);
    // attempt to push item to queue
    ev.pin = input_number;
    ev.value = rising ? 1 : 0;
    err = gpio_events_push(ev);
    if (err < 0) {
        pr_info("GPIO event queue full! Item not added.\n");
    }
    //pr_info("Queueing GPIO Interrupt: %d. Input number: %d, Rising edge: %d\n", gpio_number, input_number, rising);
    // signal input processing thread to wake up
    complete(&gpio_event_added);
    // release lock
    spin_unlock_irqrestore(&gpio_events_lock, flags);

    return IRQ_HANDLED;
}

// generate sysfs label for GPIO pins
char *generate_gpio_label(int gpio_number, bool input, int label_index)
{
    char *gpio_label;
    const char *direction = input ? "IN" : "OUT";

    gpio_label = (char*)kmalloc(GPIO_LABEL_LEN, GFP_KERNEL);
    if (gpio_label != NULL) {
        gpio_labels[label_index] = gpio_label;
        memset(gpio_label, 0, GPIO_LABEL_LEN);
        snprintf(gpio_label, GPIO_LABEL_LEN, "GPIO_%s%d", direction, gpio_number);
        //pr_info("GPIO label: %s\n", gpio_label);
    }

    return gpio_label;
}

// allocate and configure a GPIO input pin
int setup_gpio_pin(int i, bool input, bool rising)
{
    int err;
    int offset;
    int gpio_number;
    char *gpio_label;
    int gpio_irq_flags;
    int irq_number;

    // calculate offset
    offset = (input ? (rising ? GPIO_INPUT_RISING_OFFSET : GPIO_INPUT_FALLING_OFFSET) : GPIO_OUTPUT_OFFSET);
    // calculate GPIO number from sequence number
    gpio_number = (input ? (rising ? GPIO_INPUT_RISING_START : GPIO_INPUT_FALLING_START) : GPIO_OUTPUT_START) + i;
    // validate GPIO number
    if (!gpio_is_valid(gpio_number)) {
        pr_info("Error: GPIO pin %d is not valid\n", gpio_number);
        return -1;
    }
    // generate GPIO label
    gpio_label = generate_gpio_label(gpio_number, input, offset + i);
    if (gpio_label == NULL) {
        pr_info("Error generating label for GPIO pin %d\n", gpio_number);
        return -1;
    }
    // request pin
    err = gpio_request(gpio_number, (const char*)gpio_label);
    if (err < 0) {
        pr_info("Error allocating GPIO pin %d: %d\n", gpio_number, err);
        return -1;
    }
    //pr_info("Saving allocated GPIO pin %d at index %d\n", gpio_number, offset + i);
    allocated_gpio_pins[offset + i] = gpio_number;
    // configure pin
    if (input) {
        err = gpio_direction_input(gpio_number);
        if (err < 0) {
            pr_info("Error configuring GPIO pin %d to input: %d\n", gpio_number, err);
            return -1;
        }
    } else {
        err = gpio_direction_output(gpio_number, 0);
        if (err < 0) {
            pr_info("Error configuring GPIO pin %d to output: %d\n", gpio_number, err);
            return -1;
        }
    }
    // register interrupt(s)
    if (input) {
        gpio_irq_flags = rising ? IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING;
        irq_number = gpio_to_irq(gpio_number);
        //pr_info("Registering %d edge interrupt %d for GPIO pin %d\n", rising, irq_number, gpio_number);
        err = request_irq(
            irq_number,
            (void*)gpio_irq_handler,
            gpio_irq_flags,
            "netgpio",
            NULL);
        if (err < 0) {
            pr_err("Error registering IRQ for GPIO pin %d\n", gpio_number);
            return -1;
        }
        //pr_info("Saving IRQ number %d at index %d\n", irq_number, offset + i);
        allocated_irqs[offset + i] = irq_number;
    }

    return 0;
}

// initialize GPIO pins
int init_gpio(void)
{
    // TODO: call cleanup on error

    int i;

    // allocate GPIO output pins
    for (i = 0; i < NUM_GPIO_OUTPUT; i++) {
        if (setup_gpio_pin(i, false, false) < 0) {
            return -1;
        }
    }
    // allocate GPIO input pins (rising edge interrupt)
    for (i = 0; i < NUM_GPIO_INPUT; i++) {
        if (setup_gpio_pin(i, true, true) < 0) {
            return -1;
        }
    }
    // allocate GPIO input pins (falling edge interrupt)
    for (i = 0; i < NUM_GPIO_INPUT; i++) {
        if (setup_gpio_pin(i, true, false) < 0) {
            return -1;
        }
    }

    return 0;
}

// release allocated GPIO pins
void free_gpio_pins(void)
{
    int i;
    int gpio_number;
    int irq_number;

    // free IRQs
    for (i = 0; i < NUM_GPIO_PINS; i++) {
        irq_number = allocated_irqs[i];
        if (irq_number > 0) {
            pr_info("Freeing IRQ %d\n", irq_number);
            free_irq(irq_number, NULL);
        }
    }
    // free GPIO pins
    for (i = 0; i < NUM_GPIO_PINS; i++) {
        gpio_number = allocated_gpio_pins[i];
        if (gpio_number > 0) {
            pr_info("Freeing GPIO pin %d\n", gpio_number);
            gpio_free(gpio_number);
        }
    }
    // free GPIO labels
    for (i = 0; i < NUM_GPIO_PINS; i++) {
        if (gpio_labels[i] != NULL) {
            kfree(gpio_labels[i]);
        }
    }
}

// attempt to receive a given number of bytes at the given offset
int receive_bytes_at_offset(int bytes, int offset, uint8_t *buffer, int flags)
{
    // return value
    int ret;
    // message and kvec for message data
    struct msghdr msg;
    struct kvec msg_data_vec;
    // zero structures
    memset(&msg, 0, sizeof(msg));
    memset(&msg_data_vec, 0, sizeof(msg_data_vec));
    // initialize message
    msg.msg_name = 0;
    msg.msg_namelen = 0;
    msg.msg_control = NULL;
    msg.msg_controllen = 0;
    msg.msg_flags = flags;
    // set up kvec based on offset
    msg_data_vec.iov_base = buffer + offset;
    msg_data_vec.iov_len = bytes;
    // receive message
    ret = kernel_recvmsg(conn, &msg, &msg_data_vec, bytes, bytes, flags);
    return ret;
}

// attempt to send the given number of bytes at the given offset
int send_bytes_at_offset(int bytes, int offset, uint8_t *buffer, int flags)
{
    // return value
    int ret;
    // message and kvec for message data
    struct msghdr msg;
    struct kvec msg_data_vec;
    // zero structures
    memset(&msg, 0, sizeof(msg));
    memset(&msg_data_vec, 0, sizeof(msg_data_vec));
    // initialize message
    msg.msg_name = 0;
    msg.msg_namelen = 0;
    msg.msg_control = NULL;
    msg.msg_controllen = 0;
    msg.msg_flags = flags;
    // set up kvec based on offset
    msg_data_vec.iov_base = buffer + offset;
    msg_data_vec.iov_len = bytes;
    // send message
    ret = kernel_sendmsg(conn, &msg, &msg_data_vec, bytes, bytes);
    return ret;
}

int send_message(int pin, int value)
{
    // return value
    int ret;
    // message buffer
    uint8_t msg[MSG_LEN];
    // bytes sent
    int bytes_sent = 0;

    // construct message
    msg[MSG_PIN_IDX] = pin;
    msg[MSG_VALUE_IDX] = value;

    do
    {
        ret = send_bytes_at_offset(MSG_LEN - bytes_sent, bytes_sent, msg, MSG_WAITALL);
        if (ret < 0) {
            return ret;
        } else if (ret == 0) {
            // connection has closed
            return 0;
        } else {
            bytes_sent += ret;
        }
    } while (bytes_sent < MSG_LEN);

    return bytes_sent;
}

// kthread function to process gpio events from queue and send messages
int send_messages(void *unused)
{
    int ret;
    unsigned long flags;
    GPIOEvent_t ev;
    bool item;

    while (1) {
        // check if kthread should stop
        if (kthread_should_stop()) {
            return 0;
        }
        // lock spinlock to access event queue
        spin_lock_irqsave(&gpio_events_lock, flags);
        // attempt to get item from queue
        if (gpio_events_empty()) {
            item = false;
        } else {
            ev = gpio_events_pop();
            item = true;
        }
        // release lock
        spin_unlock_irqrestore(&gpio_events_lock, flags);
        // if queue was empty, sleep until otherwise
        if (!item) {
            wait_for_completion(&gpio_event_added);
            // lock mutex to re_init completion - this ensures reinitialization
            // is not currently in the process of being disabled
            mutex_lock(&gpio_event_added_reinit_lock);
            // reinit completion
            if (gpio_event_added_active) {
                reinit_completion(&gpio_event_added);
            }
            // unlock mutex
            mutex_unlock(&gpio_event_added_reinit_lock);
        } else {
            pr_info("Send: Input %d changed to %d\n", ev.pin, ev.value);
            ret = send_message(ev.pin, ev.value);
            if (ret < 0) {
                pr_info("Error sending message: %d\n", ret);
            }
        }
    }

    return 0;
}

// process a received message
int process_message(uint8_t *msg)
{
    int gpio_number;
    int value;

    gpio_number = GPIO_OUTPUT_START + msg[MSG_PIN_IDX];
    value = msg[MSG_VALUE_IDX];
    pr_info("GPIO pin: %d, value: %d\n", gpio_number, value);
    gpio_set_value(gpio_number, value);

    return 0;
}

// kthread function to receive network messages
int receive_messages(void *unused)
{
    // return value
    int ret;
    // bytes to receive at offset
    int bytes_received;
    // message buffer
    uint8_t msg_data[MSG_LEN];

    // zero message data buffer
    memset(msg_data, 0, sizeof(msg_data));

    while (1) {
        // initialize bytes needed and offset in buffer
        bytes_received = 0;
        do
        {
            // check if thread should exit
            if (kthread_should_stop()) {
                return 0;
            }
            // attempt to receive required number of bytes
            ret = receive_bytes_at_offset(MSG_LEN - bytes_received, bytes_received, msg_data, MSG_WAITALL);
            //pr_info("Received bytes: %d\n", ret);
            if (ret < 0) {
                // handle errors    
                pr_info("Error receiving bytes: %d\n", ret);
            } else {
                bytes_received += ret;
            }
        } while (bytes_received < MSG_LEN);
        // process and handle message
        process_message(msg_data);
    }

    return 0;
}

// kthread function to connect to server on initialized socket
int connect_to_server(void *unused)
{
    int i;
    int ret;
    struct sockaddr_in saddr;
    uint8_t ip_octets[4] = {127,0,0,1};
    // uint8_t ip_octets[4] = {192,168,1,106};

    // set up address/port to connect to
    memset(&saddr, 0, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_port = htons(PORT);
    saddr.sin_addr.s_addr = htonl(format_ip_address(ip_octets));

    // attempt to connect to server
    ret = kernel_connect(conn, (struct sockaddr*)&saddr, sizeof(saddr), O_RDWR);
    // keep retrying after delay if connection failed
    for (i = 0; ret < 0 && ret != -EINPROGRESS; i++) {
        msleep_interruptible(calculate_delay(i));
        if (kthread_should_stop()) {
            return 0;
        }
        ret = kernel_connect(conn, (struct sockaddr*)&saddr, sizeof(saddr), O_RDWR);
    }
    // connection successful
    pr_info("Connection successful!\n");

    // launch kthread for receiving messages
    receive_messages_kthread = kthread_run(receive_messages, NULL, "netgpio-recv");
    // launch kthread for enqueuing events
    //enqueue_gpio_events_kthread = kthread_run(enqueue_gpio_events, NULL, "netgpio-enqueue");
    // launch kthread for sending messages
    send_messages_kthread = kthread_run(send_messages, NULL, "netgpio-send");

    // wait for signal to exit
    while (!kthread_should_stop()) {
        msleep_interruptible(1000);
    }

    return 0;
}

static int __init start_netgpio(void)
{
    int err;

    pr_info("Loading netGPIO...\n");

    // initialize GPIO pins
    err = init_gpio();
    if (err < 0) {
        pr_info("Error initializing GPIO pins!\n");
        return -1;
    }
    // initialize socket
    err = init_socket();
    if (err < 0) {
        pr_info("Error initializing socket!\n");
        return -1;
    }
    // launch kthread to connect to server
    connect_kthread = kthread_run(connect_to_server, NULL, "netgpio-connect");

    return 0;
}

static void __exit end_netgpio(void)
{
    printk(KERN_INFO "Unloading netGPIO...\n");
    // stop the connection thread
    if (connect_kthread != NULL) {
        kthread_stop(connect_kthread);
    }
    // close socket, causing all calls to recvmsg 
    // on the socket to return 0 immediately
    if (conn != NULL) {
        kernel_sock_shutdown(conn, SHUT_RDWR);   
    }
    // stop the receiver thread once it wakes up,
    // which it will immediately now the socket is closed
    if (receive_messages_kthread != NULL) {
        kthread_stop(receive_messages_kthread);
    }

    // acuquire lock to prevent completion reinit
    mutex_lock(&gpio_event_added_reinit_lock);
    // signal completion to prevent thread waiting again
    complete_all(&gpio_event_added);
    // prevent reinitialization of completion
    gpio_event_added_active = false; 
    // release lock - at this point, the thread should never sleep
    // waiting for completion again
    mutex_unlock(&gpio_event_added_reinit_lock);
    // at this point, waiting is disabled so thread can exit on next iteration
    if (send_messages_kthread != NULL) {
        kthread_stop(send_messages_kthread);
    }

    // release the socket
    if (conn != NULL) {
        sock_release(conn);
    }
    // release GPIO pins
    free_gpio_pins();
}

module_init(start_netgpio);
module_exit(end_netgpio);