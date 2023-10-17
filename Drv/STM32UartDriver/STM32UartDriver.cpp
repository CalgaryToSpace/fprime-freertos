// ======================================================================
// \title  STM32UartDriverImpl.cpp
// \author tcanham
// \brief  cpp file for STM32UartDriver component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================

#include <unistd.h>
#include <Drv/STM32UartDriver/STM32UartDriver.hpp>
#include <Os/TaskString.hpp>

#include "Fw/Types/BasicTypes.hpp"

#include <fcntl.h>
#include <termios.h>
#include <cerrno>

#include "stm32l4xx_hal.h"

//#include <cstdlib>
//#include <cstdio>
//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#define DEBUG_PRINT(x, ...)

namespace Drv {

// ----------------------------------------------------------------------
// Construction, initialization, and destruction
// ----------------------------------------------------------------------

STM32UartDriver ::STM32UartDriver(const char* const compName)
    : STM32UartDriverComponentBase(compName), m_allocationSize(-1),  m_device("NOT_EXIST"), m_quitReadThread(false) {
}

void STM32UartDriver ::init(const NATIVE_INT_TYPE instance) {
  STM32UartDriverComponentBase::init(instance);
}

bool STM32UartDriver::open(const char* const device,
                           UartBaudRate baud,
                           UartFlowControl fc,
                           UartParity parity,
                           NATIVE_INT_TYPE allocationSize) {
    FW_ASSERT(device != nullptr);
    this->m_allocationSize = allocationSize;

    this->m_device = device;

    DEBUG_PRINT("Opening UART device %s\n", device);

    // TODO init UART

    if (false /* TODO ERROR */) {
        DEBUG_PRINT("open UART device %s failed.\n", device);
        Fw::LogStringArg _arg = device;
        Fw::LogStringArg _err = strerror(errno);
        // this->log_WARNING_HI_OpenError(_arg, this->m_fd, _err);
        return false;
    } else {
        DEBUG_PRINT("Successfully opened UART device %s fd %d\n", device, fd);
    }

    // All done!
    Fw::LogStringArg _arg = device;
    this->log_ACTIVITY_HI_PortOpened(_arg);
    if (this->isConnected_ready_OutputPort(0)) {
        this->ready_out(0); // Indicate the driver is connected
    }
    return true;
}

STM32UartDriver ::~STM32UartDriver() {
    DEBUG_PRINT("Closing UART device\n");
    // TODO Turn off UART stuff
}

// ----------------------------------------------------------------------
// Handler implementations for user-defined typed input ports
// ----------------------------------------------------------------------

Drv::SendStatus STM32UartDriver ::send_handler(const NATIVE_INT_TYPE portNum, Fw::Buffer& serBuffer) {
    Drv::SendStatus status = Drv::SendStatus::SEND_OK;
    if (serBuffer.getData() == nullptr || serBuffer.getSize() == 0) {
        status = Drv::SendStatus::SEND_ERROR;
    } else {
        unsigned char *data = serBuffer.getData();
        NATIVE_INT_TYPE xferSize = serBuffer.getSize();

        // TODO UART write
        // NATIVE_INT_TYPE stat = ::write(this->m_fd, data, xferSize);

        if (-1 == stat || stat != xferSize) {
          Fw::LogStringArg _arg = this->m_device;
          this->log_WARNING_HI_WriteError(_arg, stat);
          status = Drv::SendStatus::SEND_ERROR;
        }
    }
    // Deallocate when necessary
    if (isConnected_deallocate_OutputPort(0)) {
        deallocate_out(0, serBuffer);
    }
    return status;
}

void STM32UartDriver ::serialReadTaskEntry(void* ptr) {
    FW_ASSERT(ptr != nullptr);
    Drv::RecvStatus status = RecvStatus::RECV_ERROR;
    STM32UartDriver* comp = reinterpret_cast<STM32UartDriver*>(ptr);
    while (!comp->m_quitReadThread) {
        Fw::Buffer buff = comp->allocate_out(0, comp->m_allocationSize);

        // On failed allocation, error and deallocate
        if (buff.getData() == nullptr) {
            Fw::LogStringArg _arg = comp->m_device;
            comp->log_WARNING_HI_NoBuffers(_arg);
            status = RecvStatus::RECV_ERROR;
            comp->recv_out(0, buff, status);
            // to avoid spinning, wait 50 ms
            Os::Task::delay(50);
            continue;
        }

        //          timespec stime;
        //          (void)clock_gettime(CLOCK_REALTIME,&stime);
        //          DEBUG_PRINT("<<< Calling dsp_relay_uart_relay_read() at %d %d\n", stime.tv_sec, stime.tv_nsec);

        int stat = 0;

        // Read until something is received or an error occurs. Only loop when
        // stat == 0 as this is the timeout condition and the read should spin
        while ((stat == 0) && !comp->m_quitReadThread) {
            stat = ::read(comp->m_fd, buff.getData(), buff.getSize());
        }
        buff.setSize(0);

        // On error stat (-1) must mark the read as error
        // On normal stat (>0) pass a recv ok
        // On timeout stat (0) and m_quitReadThread, error to return the buffer
        if (stat == -1) {
            Fw::LogStringArg _arg = comp->m_device;
            comp->log_WARNING_HI_ReadError(_arg, stat);
            status = RecvStatus::RECV_ERROR;
        } else if (stat > 0) {
            buff.setSize(stat);
            status = RecvStatus::RECV_OK;
        } else {
            status = RecvStatus::RECV_ERROR;
        }
        comp->recv_out(0, buff, status);
    }
}

void STM32UartDriver ::startReadThread(NATIVE_UINT_TYPE priority,
                                       NATIVE_UINT_TYPE stackSize,
                                       NATIVE_UINT_TYPE cpuAffinity) {
    Os::TaskString task("SerReader");
    Os::Task::TaskStatus stat =
        this->m_readTask.start(task, serialReadTaskEntry, this, priority, stackSize, cpuAffinity);
    FW_ASSERT(stat == Os::Task::TASK_OK, stat);
}

void STM32UartDriver ::quitReadThread() {
    this->m_quitReadThread = true;
}

Os::Task::TaskStatus STM32UartDriver ::join(void** value_ptr) {
    return m_readTask.join(value_ptr);
}

}  // end namespace Drv
