//
// Created by pavel, uki and ivi on 30.12.21.
//

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/delay.h>

MODULE_LICENSE("Dual BSD/GPL");

/*
NOTE: Check Broadcom BCM8325 datasheet, page 91+
NOTE: GPIO Base address is set to 0x7E20 0000,
       but it is VC CPU BUS address, while the
       ARM physical address is 0x3F20 0000, what
       can be seen in pages 5-7 of Broadcom
       BCM8325 datasheet, having in mind that
       total system ram is 0x3F000000 (1GB - 16MB)
       instead of 0x20000000 (512 MB)
*/

//BAZNE ADRESE
#define BCM2708_PERI_BASE    (0x3F000000)                                                                               //BAZA PERIFERIJA - FIZICKA ADRESA
#define GPIO_BASE            (BCM2708_PERI_BASE + 0x200000)                                                             //GPIO BAZA
#define PWM_BASE             (BCM2708_PERI_BASE + 0x20C000)                                                             //PWM BAZA
#define CLOCK_BASE           (BCM2708_PERI_BASE + 0x101000)                                                             //CLOCK BAZA
#define GPIO_ADDR_SPACE_LEN  (0xB4)
#define PWM_ADDR_SPACE_LEN   (0x28)
#define CLOCK_ADDR_SPACE_LEN (0xC4)

//DEFINISE SE OFFSET ZA SVAKI OD REGISTARA KOJI SE KORISTI

//GPIO REGISTRI

//Handle GPIO: 0-9
/* GPIO Function Select 0. */
#define GPFSEL0_OFFSET (0x00000000)

//Handle GPIO: 10-19
/* GPIO Function Select 1. */
#define GPFSEL1_OFFSET (0x00000004)

//Handle GPIO: 20-29
/* GPIO Function Select 2. */
#define GPFSEL2_OFFSET (0x00000008)

//Handle GPIO: 30-39
/* GPIO Function Select 3. */
#define GPFSEL3_OFFSET (0x0000000C)

//Handle GPIO: 40-49
/* GPIO Function Select 4. */
#define GPFSEL4_OFFSET (0x00000010)

//Handle GPIO: 50-53
/* GPIO Function Select 5. */
#define GPFSEL5_OFFSET (0x00000014)

//PWM REGISTRI

/* Control register */
#define CTL_OFFSET  (0x00000000)

/* Range1 register */
#define RNG1_OFFSET (0x00000010)

/* Data1 register */
#define DAT1_OFFSET (0x00000014)

/* Range2 register */
#define RNG2_OFFSET (0x00000020)

/* Data2 register */
#define DAT2_OFFSET (0x00000024)

/* Pwm Clock Control register */
#define PWM_CLOCK_CONTROL   (0x000000A0)

/* Pwm Clock Divider register */
#define PWM_CLOCK_DIVIDER   (0x000000A4)

typedef enum {ALT_FUN_5_OFF = 0, ALT_FUN_5_ON = 1} STATE;

typedef enum {CHANNEL_0 = 0, CHANNEL_1 = 1} CHANNEL;

/* GPIO pins available on connector p1. */
//KORISTIM 26 GPIO PINOVA, SVI OSTALI SU 3V3, Ground, ID_SD, 5V, ID_SC
#define GPIO_02 (2)
#define GPIO_03 (3)
#define GPIO_04 (4)
#define GPIO_05 (5)
#define GPIO_06 (6)
#define GPIO_07 (7)
#define GPIO_08 (8)
#define GPIO_09 (9)
#define GPIO_10 (10)
#define GPIO_11 (11)
#define GPIO_12 (12)
#define GPIO_13 (13)
#define GPIO_14 (14)
#define GPIO_15 (15)
#define GPIO_16 (16)
#define GPIO_17 (17)
#define GPIO_18 (18)
#define GPIO_19 (19)
#define GPIO_20 (20)
#define GPIO_21 (21)
#define GPIO_22 (22)
#define GPIO_23 (23)
#define GPIO_24 (24)
#define GPIO_25 (25)
#define GPIO_26 (26)
#define GPIO_27 (27)

//KONSTANTE
#define MICURI 80
#define DIVIDER 188
#define BCM_PASSWORD (0x5A << 24)

//FUNKCIJE KOJE KORISTI DRAJVER
int  gpio_driver_init(void);
void gpio_driver_exit(void);

//FUNKCIJE KOJE KORISTI TESTNA APLIKACIJA
static int     gpio_driver_open(struct inode *, struct file *);
static int     gpio_driver_release(struct inode *, struct file *);
static ssize_t gpio_driver_read(struct file *, char *, size_t , loff_t *);
static ssize_t gpio_driver_write(struct file *, const char *, size_t , loff_t *);

// Structure that declares the usual file access functions.
struct file_operations gpio_driver_fops =
        {
                open    :   gpio_driver_open,
                release :   gpio_driver_release,
                read    :   gpio_driver_read,
                write   :   gpio_driver_write
        };

// Declaration of the init and exit functions.
module_init(gpio_driver_init);
module_exit(gpio_driver_exit);

//GLOBALNE PROMENLJIVE
static int data1  = 30;
static int data2  = 30;

//module_param je makro koji se koristi za popunjavanje promenljivih sa datim vrednostima. To cini insmod npr. ./insmod mymodule.ko data1 = nesto data2 = nesto
//koristi 3 argumenta: ime_promenljive, tip_promenljive, dozvola
//MODULE_PARM_DESC je deskriptor koji opisuje promenljive koje modul moze da koristi
//koristi 2 argumenta: ime_promenljive, opis_promenljive
module_param(data1, int, 0000);
MODULE_PARM_DESC(data1, "x-axis servo data, GPIO_18");

module_param(data2, int, 0000);
MODULE_PARM_DESC(data2, "y-axis servo data, GPIO_19");

/* Buffer to store data. */
char* gpio_driver_buffer;

/* Major number. */
unsigned int gpio_driver_major;

/* Virtual address where the physical GPIO address is mapped */
void* virt_pwm_base;
void* virt_gpio_base;
void* virt_pwm_clock_base;

/*
 * GetGPFSELReg function
 *  Parameters:
 *    pin    - number of GPIO pin;
 *    return - GPFSELn offset from GPIO base address, for containing desired pin control
 *  Operation:
 *    Based on the passed GPIO pin number, finds the corresponding GPFSELn reg and
 *    returns its offset from GPIO base address.
 */
unsigned int GetGPFSELReg(char pin)
{
    unsigned int addr;

    if(pin >= 0 && pin <10)
        addr = GPFSEL0_OFFSET;
    else if(pin >= 10 && pin <20)
        addr = GPFSEL1_OFFSET;
    else if(pin >= 20 && pin <30)
        addr = GPFSEL2_OFFSET;
    else if(pin >= 30 && pin <40)
        addr = GPFSEL3_OFFSET;
    else if(pin >= 40 && pin <50)
        addr = GPFSEL4_OFFSET;
    else /*if(pin >= 50 && pin <53) */
        addr = GPFSEL5_OFFSET;

    return addr;
}

/*
 * GetGPIOPinOffset function
 *  Parameters:
 *    pin    - number of GPIO pin;
 *    return - offset of the pin control bit, position in control registers
 *  Operation:
 *    Based on the passed GPIO pin number, finds the position of its control bit
 *    in corresponding control registers.
 */
char GetGPIOPinOffset(char pin)
{
    if(pin >= 0 && pin <10)
        pin = pin;
    else if(pin >= 10 && pin <20)
        pin -= 10;
    else if(pin >= 20 && pin <30)
        pin -= 20;
    else if(pin >= 30 && pin <40)
        pin -= 30;
    else if(pin >= 40 && pin <50)
        pin -= 40;
    else /*if(pin >= 50 && pin <53) */
        pin -= 50;

    return pin;
}

/*
 * SetAltFun0 function
 *  Parameters:
 *   pin   - number of GPIO pin which is going to be set to use alternate function 0 or to stop using it;
 *   state - ALT_FUN_0_ON or ALT_FUN_0_OFF
 *  Operation:
 *   Sets the desired GPIO pin to be used as PWM or not, in out solution, this can be used only with GPIO_12 and GPIO_13 pins
 */
void setAltFun5(char pin, STATE state){
    //PROVERI OVU CELU FUNKCIJU
    char gpio_pin = pin;
    unsigned int reg_offset;
    char pin_offset;
    unsigned int tmp;
    unsigned int mask;

    /* Get base address of function selection register. */
    //DOBIJAMO OFFSET FSEL REGISTRA OD INTERESA
    reg_offset = GetGPFSELReg(gpio_pin);

    /* Calculate gpio pin offset. */
    //DOBIJAMO POZICIJU DATOG PINA U REGISTRU, NPR. ZA GPIO_12 CE BITI 2
    pin_offset = GetGPIOPinOffset(gpio_pin);

    //POZICIONIRAJ SE NA ODGOVARAJUCI REGISTAR, BEZ OFFSETA
    tmp = ioread32(virt_gpio_base + reg_offset);

    if(state == ALT_FUN_5_ON){
        //STAVI 100 ZA ALT FUN 0
        //STAVI 010 ZA ALT FUN 5

        mask = ~(0x1 << (pin_offset*3 + 2));
        tmp &= mask;

        mask = 0x1 << (pin_offset*3 + 1);
        tmp |= mask;

        mask = ~(0x1 << (pin_offset*3));
        tmp &= mask;
    }else{
        mask = ~(0x1 << (pin_offset*3 + 1));
        tmp &= mask;
    }

    //ZAPISUJEM IZMENU
    iowrite32(tmp, virt_gpio_base + reg_offset);
    printk(KERN_INFO "FJA --- Sadrzaj registra FSEL1 je %x", tmp);
}

/*
 * CTL - Control Register
 *
 * setCTL function
 *  Parameters:
 *   kanal - u zavisnosti od kanala menjaju se odgovarajuci biti koji omogucuju MS transmisiju
 *  Operation:
 *   Postavlja registar CTL u odgovarajuce stanje, tj. menja pojedine bite od interesa.
 */
void setCTL(CHANNEL kanal){
    unsigned int ctl;

    ctl = ioread32(virt_pwm_base + CTL_OFFSET);

    if(kanal == CHANNEL_0){
        ctl |= 0x80;    //koristim MS transmisiju
        ctl |= 0x1;     //kanal 0 enable
    }
    else{
        ctl |= 0x8000;      //koristim MS transmisiju
        ctl |= 0x100;       //kanal 1 enable
    }

    iowrite32(ctl, virt_pwm_base + CTL_OFFSET);
    printk(KERN_INFO "FJA --- Sadrzaj registra CTL je %x", ctl);
}

/*
 * clear CTL function
 *  Parameters:
 *   nep - nepotreban parametar koji prebacujemo kako bismo izbegli warning-e
 *  Operation:
 *   Postavlja registar CTL u inicijalno stanje, tj. menja pojedine bite od interesa.
 */
void clearCTL(void* nep) {
    unsigned int ctl;

    ctl = 0x00000000;

    iowrite32(ctl, virt_pwm_base + CTL_OFFSET);
}

/*
 * setRNGi function
 *  Parameters:
 *   kanal - u zavisnosti od kanala menja se odgovarajuci range registar
 *  Operation:
 *   Postavlja registre RNG1 i RNG2 u odgovarajuce stanje, mi koristimo vrednost 375 za ovaj registar
 */
void setRNGi(CHANNEL kanal){
    unsigned int tmp;

    //1024
    tmp = 1024;

    if(kanal == CHANNEL_0){
        iowrite32(tmp, virt_pwm_base + RNG1_OFFSET);
        printk(KERN_INFO "FJA --- Sadrzaj registra RNG1 je %x", tmp);
    }
    else{
        iowrite32(tmp, virt_pwm_base + RNG2_OFFSET);
        printk(KERN_INFO "FJA --- Sadrzaj registra RNG2 je %x", tmp);
    }
}

/*
 * clearRNGi function
 *  Parameters:
 *   nep - nepotreban parametar koji prebacujemo kako bismo izbegli warning-e
 *  Operation:
 *   Postavlja registre RNG1 i RNG2 u inicijalno stanje, mi koristimo vrednost 32 za ovaj registar
 */
void clearRNGi(void* nep) {

    unsigned int tmp;

    tmp = 0x00000020;

    iowrite32(tmp, virt_pwm_base + RNG1_OFFSET);
    iowrite32(tmp, virt_pwm_base + RNG2_OFFSET);
}

/*
 * setDAT1 function
 *  Parameters:
 *   nep - nepotreban parametar koji prebacujemo kako bismo izbegli warning-e
 *  Operation:
 *   Menja se vrednost DATA registra u zavisnosti od ugla1
 */
void setDATi(CHANNEL kanal) {
    unsigned int tmp;

    if(kanal == CHANNEL_0){
        tmp = data1;
        iowrite32(tmp, virt_pwm_base + DAT1_OFFSET);
        printk(KERN_INFO "FJA --- Sadrzaj registra DAT1 je %x", tmp);
    }
    else{
        tmp = data2;
        iowrite32(tmp, virt_pwm_base + DAT2_OFFSET);
        printk(KERN_INFO "FJA --- Sadrzaj registra DAT2 je %x", tmp);
    }
}

/*
 * clearDATi function
 *  Parameters:
 *   nep - nepotreban parametar koji prebacujemo kako bismo izbegli warning-e
 *  Operation:
 *   Resetuje sadrzaj DATA registra
 */
void clearDATi(void* nep) {

    unsigned int tmp;

    tmp = 0x00000000;

    iowrite32(tmp, virt_pwm_base + DAT1_OFFSET);
    iowrite32(tmp, virt_pwm_base + DAT2_OFFSET);
}

/*
 * setCLK function
 *  Parameters:
 *   divider - parametar sa kojim delimo inicijalnu frekvenciju pwm clock-a koja iznosi 19.2MHz
 *  Operation:
 *   Postavlja frekvenciju pwm clocka na zeljenu
 */
void setCLK(unsigned int divider){

    unsigned int tmp;
    unsigned int int_div;
    unsigned int reset;

    divider &= 0xfff;

    tmp = ioread32(virt_pwm_clock_base + PWM_CLOCK_CONTROL);
    printk(KERN_INFO "FJA --- Sadrzaj registra PWM_CLOCK_CONTROL je %x", tmp);

    //STOP PWM CLOCK, NAKON OVOG KORAKA SE BUSY POSTAVLJA NA 0
    //UNOSIM 5A000001
    iowrite32( (BCM_PASSWORD | 0x01), virt_pwm_clock_base + PWM_CLOCK_CONTROL);
    mdelay(110);

    //CEKAJ DA BUSY BUDE NA 0, STO JE NEOPHODNO ZBOG SETOVANJA DIVIDERA
    while( (ioread32(virt_pwm_clock_base + PWM_CLOCK_CONTROL) & 0x80) != 0 ) {
        printk(KERN_INFO "Cekam da BUSY bude 0");
        mdelay(1);
    }

    //DELITELJ SE UBACUJE U DEO REGISTRA DIVIDER KOJI ODGOVARA INT DELU DIVIDERA
    iowrite32( (BCM_PASSWORD | (divider << 12) ), virt_pwm_clock_base + PWM_CLOCK_DIVIDER);
    int_div = ioread32(virt_pwm_clock_base + PWM_CLOCK_DIVIDER);
    printk(KERN_INFO "FJA --- Sadrzaj registra PWM_CLOCK_DIVIDER je %x", int_div);

    //ENABLE PWM CLOCK I SOURCE = OSC
    iowrite32( (BCM_PASSWORD | 0x11), virt_pwm_clock_base + PWM_CLOCK_CONTROL);
    reset = ioread32(virt_pwm_clock_base + PWM_CLOCK_CONTROL);
    printk(KERN_INFO "FJA --- Sadrzaj registra PWM_CLOCK_CONTROL je %x", reset);
}

/*
 * stringToIntfunction
 *  Parameters:
 *   pomocna - parametar koji predstavlja pocetni string
 *  Operation:
 *   Parsira ulazni string u int i vraca ga kao povratnu vrednost
 */
int stringToInt(char* pomocna){
    char* x = pomocna;
    int duzina = strlen(x);

    int rezultat = 0;
    int i;

    for(i = 0; i < duzina; i++){
        rezultat = rezultat * 10 + ( x[i] - '0' );
    }

    return rezultat;
}

/*
 *  Initialization:
 *  1. Register device driver
 *  2. Allocate buffer
 *  3. Initialize buffer
 *  4. Map GPIO Physical address space to virtual address
 *  5. Initialize PWM pins
 *  6. Change Clock frequency
 */

int gpio_driver_init(void){
    int result = -1;

    //KERN_INFO JE LOG LEVEL, SESTI PO REDU
    printk(KERN_INFO "Inserting gpio_driver module\n");

    /*1 Registering device. */
    //FUNKCIJA REGISTRUJE CHARDEV
    //1. parametar - unsigned int major - major broj ili 0 za dinamicku alokaciju
    //2. parametar - constr char* name - ime uredjaja
    //3. parametar - const struct file_operations* fops - operacije koje koristi ovaj uredjaj
    // AKO JE MAJOR == 0 --> FUNKCIJA VRACA MAJOR BROJ CHARDEVA KOJI SE REGISTRUJE
    // AKO JE MAJOR > 0, FUNKCIJA CE POKUSATI DA ZADRZI UREDJAJ SA DATIM MAJOR BROJEM I VRATI 0 AKO JE USPESNA
    result = register_chrdev(0, "gpio_driver", &gpio_driver_fops);
    if (result < 0)
    {
        printk(KERN_INFO "gpio_driver: cannot obtain major number %d\n", gpio_driver_major);
        return result;
    }

    //AKO JE USPESNO REGISTROVAN CHARDEV, POPUNJAVA SE POLJE MAJOR
    gpio_driver_major = result;
    printk(KERN_INFO "gpio_driver major number is %d\n", gpio_driver_major);

    /*2 Allocating memory for the buffer. */
    //POPUNJAVA SE BAFER DRAJVERA
    //1. parametar - velicina potrebne memorije u bajtima
    //2. parametar - tip memorije koja se alocira - POSTOJI DOSTA FLAGOVA ZA 2. PARAMETAR, OVAJ OZNACAVA - ALOKACIJA NORMALNOG KERNEL RAM-A.
    gpio_driver_buffer = kmalloc(MICURI, GFP_KERNEL);

    //U SLUCAJU NEUSPELE ALOKACIJE MEMORIJE, SKACI NA LABELU FAIL_NO_MEM
    if (!gpio_driver_buffer)
    {
        result = -ENOMEM;
        goto fail_no_mem;
    }

    /*3 Initialize data buffer. */
    //POPUNI BAFER SA SVIM NULL KARAKTERIMA
    memset(gpio_driver_buffer, '\0', MICURI);

    /*4 map the GPIO register space from PHYSICAL address space to virtual address space */

    //MAPIRANJE FIZICKE ADRESE REGISTRA NA VIRTUELNU ADRESU U OKVIRU KERNELA
    //SUPROTNA OPERACIJA JE IOUNMAP()
    //1. parametar - fizicka adresa registra
    //2. parametar - velicina
    //POVRATNA VREDNOST JE VIRTUELNA BAZA KOJA SE KORISTI U FUNKCIJAMA
    //U SLUCAJU NEUSPELE ALOKACIJE MEMORIJE, SKACI NA LABELU FAIL_NO_VIRT_MEM

    virt_gpio_base = ioremap(GPIO_BASE, GPIO_ADDR_SPACE_LEN);
    if(!virt_gpio_base)
    {
        result = -ENOMEM;
        goto fail_no_virt_mem;
    }

    virt_pwm_clock_base = ioremap(CLOCK_BASE, CLOCK_ADDR_SPACE_LEN);
    if(!virt_pwm_clock_base)
    {
        result = -ENOMEM;
        goto fail_no_virt_mem;
    }

    virt_pwm_base = ioremap(PWM_BASE, PWM_ADDR_SPACE_LEN);
    if(!virt_pwm_base)
    {
        result = -ENOMEM;
        goto fail_no_virt_mem;
    }

    /*5 Initialize PWM pins. */
    //STAVLJAMO PINOVE U FUNKCIJU U KOJOJ SU NAMENJENI ZA PWM OUTPUT
    setAltFun5(GPIO_18 , ALT_FUN_5_ON);
    setAltFun5(GPIO_19 , ALT_FUN_5_ON);

    setCLK(DIVIDER);

    setCTL(CHANNEL_0);
    setCTL(CHANNEL_1);

    setRNGi(CHANNEL_0);
    setRNGi(CHANNEL_1);

    setDATi(CHANNEL_0);
    setDATi(CHANNEL_1);

    return 0;

    //GOTO LABELE, DO NJIH SE DOLAZI U SLUCAJEVIMA NEUSPELOG RADA PROGRAMA
    fail_no_virt_mem:
    // Unmap Physical address space.
    if (virt_pwm_base)
    {
        iounmap(virt_pwm_base);
    }

    if (virt_gpio_base)
    {
        iounmap(virt_gpio_base);
    }

    if (virt_pwm_clock_base)
    {
        iounmap(virt_pwm_clock_base);
    }

    // Freeing buffer gpio_driver_buffer.
    if (gpio_driver_buffer)
    {
        kfree(gpio_driver_buffer);
    }

    fail_no_mem:
    /* Freeing the major number. */
    unregister_chrdev(gpio_driver_major, "gpio_driver");

    return result;
}


/*
 * Cleanup:
 *  1. release PWM pins (clear all outputs, set all as inputs and pull-none to minimize the power consumption)
 *  2. Unmap PWM Physical address space from virtual address
 *  3. Free buffer
 *  4. Unregister device driver
 */
void gpio_driver_exit(void)
{
    int nep = 7;

    printk(KERN_INFO "Removing pwm_driver module\n");

    /* 1. release PWM pins */
    clearDATi(&nep);

    clearRNGi(&nep);

    clearCTL(&nep);

    //STAVLJAMO PINOVE U FUNKCIJU U KOJOJ SU NAMENJENI ZA GPIO
    setAltFun5(GPIO_18 , ALT_FUN_5_OFF);
    setAltFun5(GPIO_19 , ALT_FUN_5_OFF);

    /*2 Unmap GPIO Physical address space. */
    if (virt_gpio_base)
    {
        iounmap(virt_gpio_base);
    }

    if (virt_pwm_base)
    {
        iounmap(virt_pwm_base);
    }

    /*3 Freeing buffer gpio_driver_buffer. */
    if (gpio_driver_buffer)
    {
        kfree(gpio_driver_buffer);
    }

    /*4 Freeing the major number. */
    unregister_chrdev(gpio_driver_major, "gpio_driver");
}

/* File open function. */
static int gpio_driver_open(struct inode *inode, struct file *filp)
{
    /* Initialize driver variables here. */
    /* Reset the device here. */
    /* Success. */
    return 0;
}

/* File close function. */
static int gpio_driver_release(struct inode *inode, struct file *filp)
{
    /* Success. */
    return 0;
}

/*
 * File read function
 *  Parameters:
 *   filp  - a type file structure;
 *   tmp   - a buffer, from which the user space function (fread) will read;
 *   len - a counter with the number of bytes to transfer, which has the same
 *           value as the usual counter in the user space function (fread);
 *   f_pos - a position of where to start reading the file;
 *  Operation:
 *   The pwm_driver_read function transfers data from the driver buffer (gpio_driver_buffer)
 *   to user space with the function copy_to_user.
 */
static ssize_t gpio_driver_read(struct file* filp, char* tmp, size_t len, loff_t* f_pos){
    char data1_str[4];
    char data2_str[4];

    /* Size of valid data in gpio_driver - data to send in user space. */
    int data_size = 0;

    /* Reset memory. */
    memset(gpio_driver_buffer, '\0', MICURI);

    snprintf(data1_str, 4, "%d", data1);
    snprintf(data2_str, 4, "%d", data2);

    //gpio_driver_buffer = data1 data2
    strcat(gpio_driver_buffer, data1_str);
    strcat(gpio_driver_buffer, " ");
    strcat(gpio_driver_buffer, data2_str);

    /* Get size of valid data. */
    data_size = strlen(gpio_driver_buffer);

    /* Send data to user space. */
    //KOPIRA BLOK PODATAKA U USER SPACE
    //1. parametar - adresa bafera gde se kopiraju podaci
    //2.parametar - adresa bafera sa koga se kopiraju podaci
    //3.parametar - duzina bafera
    if (copy_to_user(tmp, gpio_driver_buffer, data_size) != 0)
    {
        return -EFAULT;
    }
    else
    {
        (*f_pos) += data_size;

        return data_size;
    }
}

/*
 * File write function
 *  Parameters:
 *   filp  - a type file structure;
 *   tmp   - a buffer in which the user space function (fwrite) will write;
 *   MICURI - a counter with the number of bytes to transfer, which has the same
 *           values as the usual counter in the user space function (fwrite);
 *   f_pos - a position of where to start writing in the file;
 *  Operation:
 *   The function copy_from_user transfers the data from user space to kernel space.
 */
static ssize_t gpio_driver_write(struct file* filp, const char *tmp, size_t len, loff_t *f_pos) {
    int servo_id = 0;
    int data = 0;
    int i = 0;
    char* pomocna;
    char* pokazivac;
    pokazivac = gpio_driver_buffer;

    /* Reset memory. */
    memset(gpio_driver_buffer, '\0', MICURI);

    /* Get data from user space. [tmp -> gpio_driver_buffer] */
    if (copy_from_user(gpio_driver_buffer, tmp, len) != 0) {
        //DOSLO DO GRESKE U KOPIRANJU, JER FUNKCIJA VRACA BROJ BAJTA KOJI SU NEUSPELO KOPIRANI
        return -EFAULT;
    } else {

        while ((pomocna = strsep(&gpio_driver_buffer, " ")) != NULL) {
            if (i == 0) {
                servo_id = pomocna[0] - '0';
            } else if (i == 1) {
                data = stringToInt(pomocna);
            }
            i++;
        }
        i = 0;

        gpio_driver_buffer = pokazivac;

        //sad smo popunili servo_id i angle

        switch (servo_id) {
            case 1:
                data1 = data;
                setDATi(CHANNEL_0);
                break;

            case 2:
                data2 = data;
                setDATi(CHANNEL_1);
                break;

            default:
                printk(KERN_INFO "MAJMUNEEEE");
                break;
        }

        return len;
    }
}