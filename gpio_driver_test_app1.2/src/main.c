//
// Created by pavel and uki on 12.1.22..
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>

#define LOSMI 0.85714285714
//TODO UBACI KONSTANTU SUBE
#define MICURI 80

/*
    insmod ./gpio_driver.ko
    lsmod
    dmesg -w
    mknod /dev/gpio_driver c 242 0

    ./gpio_driver_test_app

    rm /dev/gpio_driver
    rmmod memory
*/

int preuzimanjeParametara(char* tmp);
void konverzijaUgla(char* buff);

int servo_id = 0;
int angle = 0;

int main(){
    char tmp[MICURI];
    char buff[MICURI];
    int file_desc;
    int ret_val;
    int q;

    file_desc = open("/dev/gpio_driver", O_RDWR);

    if(file_desc < 0)
    {
        printf("Error, '/dev/gpio_driver' not opened\n");
        return -1;
    }

    while(1) {
        memset(tmp, '\0', MICURI);
        memset(buff, '\0', MICURI);

        do{
            printf("Uneti zeljeni servo motor i njegov ugao = ");
            fgets(tmp, MICURI, stdin);
            q = preuzimanjeParametara(tmp);
        }while(q == 1);

        konverzijaUgla(buff);

        printf("Poruka koja se salje drajveru --- %s\n", buff);

        ret_val = write(file_desc, buff, MICURI);

        if (ret_val > 0) {
            printf("Write uspesan\n");
        }

        ret_val = read(file_desc, tmp, MICURI);
        if (ret_val > 0) {
            printf("Read 2 uspesan\n");
            printf("Vrednosti data1 i data2 registara --- %s\n", tmp);
        }
    }

    printf("KRAJ\n");

    close(file_desc);

    return 0;
}

int preuzimanjeParametara(char* tmp){
    char* pokazivac;
    pokazivac = tmp;
    char* pomocna;

    int x = 0;

    while ((pomocna = strsep(&tmp, " ")) != NULL) {
        if (x == 0) {
            servo_id = pomocna[0] - '0';
        } else if (x == 1) {
            angle = atoi(pomocna);
        }
        x++;
    }
    x = 0;

    tmp = pokazivac;

    if(servo_id < 1 || servo_id > 2 || angle < 0 || angle > 180){
        return 1;
    }

    return 0;
}

void konverzijaUgla(char* buff){
    //MAX VREDNOST ZA DATA JE 124, A MIN JE 20
    int data = 0;
    char id_str[2];
    char data_str[4];

    double pom = (double)angle / LOSMI;

    //TODO EVENTUALNO OVDE POVECAJ PRECIZNOST
    data = 30 + (int)pom;

    snprintf(id_str, 2, "%d", servo_id);
    snprintf(data_str, 4, "%d", data);

    strcat(buff, id_str);
    strcat(buff, " ");
    strcat(buff, data_str);
}