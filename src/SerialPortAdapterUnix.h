//
// Created by phil on 21/07/2018.
//

#ifndef SERIALPORTADAPTERUNIX_H
#define SERIALPORTADAPTERUNIX_H


class SerialPortAdapterUnix : public SerialPortAdapter{
public:
    int  open(const char *port) override ;
    bool setup(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control) override ;
    bool close() override ;
    int  read(uint8_t *ch) override ;
    int write(char *buf, unsigned len) override ;
    SerialPortAdapterUnix();
    ~SerialPortAdapterUnix();
protected:
    int  fd;
    pthread_mutex_t  lock;

};


#endif //SERIAL_SERIALPORTADAPTERUNIX_H
