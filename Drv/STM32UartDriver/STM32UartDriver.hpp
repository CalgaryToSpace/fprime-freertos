// ======================================================================
// \title  STM32UartDriverImpl.hpp
// \author tcanham
// \brief  hpp file for STM32UartDriver component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================

#ifndef STM32UartDriver_HPP
#define STM32UartDriver_HPP

#include <Drv/STM32UartDriver/STM32UartDriverComponentAc.hpp>
#include <Os/Mutex.hpp>
#include <Os/Task.hpp>

namespace Drv {

class STM32UartDriver : public STM32UartDriverComponentBase {
  public:
    // ----------------------------------------------------------------------
    // Construction, initialization, and destruction
    // ----------------------------------------------------------------------

    //! Construct object STM32UartDriver
    //!
    STM32UartDriver(const char* const compName /*!< The component name*/
    );

    //! Initialize object STM32UartDriver
    //!
    void init(const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
    );

    //! Configure UART parameters
    enum UartBaudRate { BAUD_9600=9600, BAUD_19200=19200, BAUD_38400=38400, BAUD_57600=57600, BAUD_115K=115200, BAUD_230K=230400, BAUD_460K=460800, BAUD_921K=921600};

    enum UartFlowControl { NO_FLOW, HW_FLOW };

    enum UartParity { PARITY_NONE, PARITY_ODD, PARITY_EVEN };

    // Open device with specified baud and flow control.
    bool open(const char* const device, UartBaudRate baud, UartFlowControl fc, UartParity parity, NATIVE_INT_TYPE allocationSize);

    //! start the serial poll thread.
    //! buffSize is the max receive buffer size
    //!
    void startReadThread(NATIVE_UINT_TYPE priority = Os::Task::TASK_DEFAULT,
                         NATIVE_UINT_TYPE stackSize = Os::Task::TASK_DEFAULT,
                         NATIVE_UINT_TYPE cpuAffinity = Os::Task::TASK_DEFAULT);

    //! Quit thread
    void quitReadThread();

    //! Join thread
    Os::Task::TaskStatus join(void** value_ptr);

    //! Destroy object STM32UartDriver
    //!
    ~STM32UartDriver();

  PRIVATE:
    // ----------------------------------------------------------------------
    // Handler implementations for user-defined typed input ports
    // ----------------------------------------------------------------------

    //! Handler implementation for serialSend
    //!
    Drv::SendStatus send_handler(NATIVE_INT_TYPE portNum, /*!< The port number*/
                                 Fw::Buffer& serBuffer);


    NATIVE_INT_TYPE m_allocationSize; //!< size of allocation request to memory manager
    const char* m_device;  //!< original device path

    //! This method will be called by the new thread to wait for input on the serial port.
    static void serialReadTaskEntry(void* ptr);

    Os::Task m_readTask;  //!< task instance for thread to read serial port

    bool m_quitReadThread;  //!< flag to quit thread
};

}  // end namespace Drv

#endif
