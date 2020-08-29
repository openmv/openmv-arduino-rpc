# OpenMV Arduino RPC Interface Library

The rpc library allows you to connect your Arduino to your OpenMV Cam, another Arduino, or computer and execute remote procedure calls and stream data at the highest speeds possible over CAN, I2C, SPI, UART (Async Serial), and USB. It is designed to be robust and fault tolerant. It automatically can recover from wires coming loose, packet corruption, timeouts, and etc.

# How to use the Library

Please look at the following scripts for how to control your OpenMV Cam from your Arduino:

* [Popular Features](examples/popular_features_as_the_controller_device_example/popular_features_as_the_controller_device_example.ino)
* [Image Transfer](examples/image_transfer_jpg_as_the_controller_device_example/image_transfer_jpg_as_the_controller_device_example.ino)

The sketches above work with scripts running onboard the OpenMV Cam. The `Popular Features` sketch shows off how to trivially use an Arduino to access powerful features onboard the OpenMV Cam.

# API

The OpenMV Arduino RPC Library has a lot going on under the hood and is designed to be extensible to work over any communication interface. To keep the documentation simple and stright forward only the public interfaces designed for most users are detailed below.

_Note that all classes in the OpenMV RPC Library are under the `openmv` namespace_

## Scratch Buffer

To prevent the use of dynamic memory on your Arduino the RPC Library uses a global scratch buffer that all RPC interfaces use. All you need to do is define the size of the scratch buffer in bytes by adding:

    openmv::rpc_scratch_buffer<256> scratch_buffer;

To your sketch where the number passed in the brackets above is the size of the scratch buffer. RPC Library packet sizes will be limited to the number of bytes in the scratch buffer above. You may call `size_t buffer_size()` on the scratch buffer to get how many bytes are in the scratch buffer.

## RPC Master

The interface library supports command and control over the following interfaces:

* CAN using the `rpc_can_master`
* I2C using the `rpc_i2c_master`
* SPI using the `rpc_spi_master`
* Hardware Serial using the `rpc_hardware_serial_uart_master`, `rpc_hardware_serial1_uart_master`, `rpc_hardware_serial2_uart_master`, `rpc_hardware_serial3_uart_master`, `rpc_hardware_serialUSB_uart_master`
* Software Serial using the `rpc_software_serial_uart_master`

CAN works using the MCP2515 over SPI with an Arduino or the internal CAN peripheral on the ESP32, I2C and SPI use the default I2C/SPI bus on your Arduino, Hardware Serial uses the Hardware Serial ports on your Arduino, and Software Serial allows you to do low speed serial over any I/O pins that support interrupts on AVR based Arduinos.

After creating any of the above objects you just have to call `begin()` on them for the interface to be enabled. Afterwhich, you can do remote procedure calls by executing `call()`. Finally, you can shutdown the interface with `end()`.

There are a few different variants of the `call()` method on your Arduino for specific situations.

* `call()`
* `call_no_args()`
* `call_no_copy()`
* `call_no_copy_no_args()`

`call()` takes the following arguments:

* A string name of the remote function you want to call (can be a `char *`, `String`, or flash string `F()`). __required__
* A pointer to the data you want the pass. __required__
* The length in bytes of the data you want to pass. __required__
* A pointer to where to put the return data. (default `NULL`)
* The length in bytes of the buffer for the return data. (default `0`)
* A flag that if `true` causes `call()` to return `false` if the return data length is zero. (default `true`)
* How many milliseconds to try to send the command. (default `1000`)
* How many milliseconds to wait for the response. (default `1000`)

On success the return value will be `true`. On failure it will be `false` or if you received a zero-length result if the above flag is set. On success the return data buffer will have the result data copied to it (up to the size of the buffer), note that there's no way to know the return data length if it's variable with `call()` - use `call_no_copy()` for variable length return data. On failure the return data buffer will be zeroed.

_The point of returning false if you receive a zero-length result is because the RPC library on the OpenMV Cam returns zero-length results when a function like "face_detection" finds no faces. This saves having to return the received data length for fixed-length data structure situations and having to write more code per RPC call._

`call()` is designed for when you want to call a remote function and have to pass arguments. `call_no_args()` drops the the required pointer and length arguments above so you don't have to pass NULL and 0 for remote function calls that take no arugments. `call_no_copy()` modifies the function signature slightly to minimize RAM use by instead returning a pointer into the scratch buffer versus copying data to the target buffer you reference using the regular `call()` method. This version of the `call()` method is for advanced users but can greatly save RAM. `call_no_copy()` takes the following arguments:

* A string name of the remote function you want to call (can be a `char *`, `String`, or flash string). __required__
* A pointer to the data you want the pass. __required__
* The length in bytes of the data you want to pass. __required__
* A pointer to a pointer to set to the address in the scratch buffer of the returned data (`void **result_data`) (default `NULL`) 
* A pointer to a length to set to the length in bytes of the return data. (`size_t *result_data_len)` (default `NULL`)
* A flag that if `true` causes `call_no_copy()` to return `false` if the return data length is zero. (default `true`)
* How many milliseconds to try to send the command. (default `1000`)
* How many milliseconds to wait for the response. (default `1000`)

Finally, `call_no_copy_no_args()` drops the the required pointer and length arguments above so you don't have to pass NULL and 0 for remote function calls that take no arugments.

The RPC Library was originally designed in Python where the above is a lot more simple as Python allows you to pass a byte array and return a byte array. However, in C++ you have to be very explict about things which is the reason for the complexity above. That said, the RPC Library allows you to do general purpose remote function calls that can pass up to 2^32-1 bytes of data (or 2^16-1 bytes on data on Arduino AVR architectures) and get 2^32-1 bytes of data (or 2^16-1 bytes on data on Arduino AVR architectures) as a result. This allows you move very rich data structures back and forth over any of the communication links below. For example:

    struct { uint16_t x, y, w, h; } face_detection_result;

    if (interface.call_no_args(F("face_detection"), &face_detection_result, sizeof(face_detection_result)))
    {
        Serial.print(F("Largest Face Detected [x="));
        Serial.print(face_detection_result.x);
        Serial.print(F(", y="));
        Serial.print(face_detection_result.y);
        Serial.print(F(", w="));
        Serial.print(face_detection_result.w);
        Serial.print(F(", h="));
        Serial.print(face_detection_result.h);
        Serial.println(F("]"));
    }

In the code above, we define a structure that we expect the OpenMV Cam to return. Then we just need to call the `face_detection` method on the OpenMV Cam (note the use the `F()` to store the string in flash to save RAM). We pass `call_no_args()` the remote function string, the address of the structure we just defined to hold the result, and then the size of that structure and the RPC Library takes care of the rest. On success `call_no_args()` returns true with the data received by executing the remote `face_detection` method on the OpenMV Cam.

And if you need to pass complex arguments this is easy too:

    int8_t thresholds[6] = {30, 100, 15, 127, 15, 127}; // LAB Color Thresholds (LMIN, LMAX, BMIN, BMAX, AMIN, AMAX)

    struct { uint16_t cx, cy; } centroid;

    if (interface.call(F("color_detection"), thresholds, sizeof(thresholds), &centroid, sizeof(centroid)))
    {
        Serial.print(F("Largest Color Detected [cx="));
        Serial.print(color_detection_result.cx);
        Serial.print(F(", cy="));
        Serial.print(color_detection_result.cy);
        Serial.println(F("]"));
    }

In this example above we're now using the `call()` method so we can pass arugments. So, we define an array of 6 color thresholds that we expect the `color_detection` function on the OpenMV Cam to support. Then, we just have to pass the array of thresholds and the size of the array to `call()`.

For how to use `call_no_copy()` and `call_no_copy_no_args()` see the [Image Transfer](examples/image_transfer_jpg_as_the_controller_device_example/image_transfer_jpg_as_the_controller_device_example.ino) sketch which makes use of this method to save RAM.

_`call_no_copy()` is the complete RPC library call function. However, it's not the most beginner friendly so the other call methods exist to reduce the amount of typing required._

Next, below are all the constructors of the interfaces you can create. Each interface supports all the `begin()`, `call()`, and `end()` logic above.

## rpc_can_master(int message_id=0x7FF, long bit_rate=250E3) 

`message_id` defines an 11-bit CAN message ID that all communication over CAN will happen using. `bit_rate` defines the speed the CAN bus will run at.

You may change the `message_id` after the object is created by using `void set_message_id(int message_id)` and you may get the current `message_id` by using `int get_message_id()`. This is useful for talking to multiple RPC devices at the same time.

## rpc_i2c_master(uint8_t slave_addr=0x12, uint32_t rate=100000)

`slave_addr` defines the I2C address of the RPC device to talk to. `rate` defines the I2C bus frequency.

You may change the `slave_addr` after the object is created by using `void set_slave_addr(uint8_t cs_pin)` and you may get the current `slave_addr` by using `uint8_t get_slave_addr()`. This is useful for talking to multiple RPC devices at the same time.

## rpc_spi_master(uint8_t cs_pin, uint32_t freq=1000000, uint8_t spi_mode=SPI_MODE2)

`cs_pin` is the CS pin to use with the default SPI bus. `freq` is the frequency to run at. `spi_mode` is the Arduino SPI mode to use (the OpenMV Cam uses SPI_MODE2 by default).

You may change the `cs_pin` after the object is created by using `void set_cs_pin(uint8_t cs_pin)` and you may get the current `cs_pin` by using `uint8_t get_cs_pin()`. This is useful for talking to multiple RPC devices at the same time.

## rpc_hardware_serial_uart_master(unsigned long baudrate=115200)

`baudrate` is the bit rate to run at. Note that depending on your Arduino you may have access to the additional `rpc_hardware_serial1_uart_master`, `rpc_hardware_serial2_uart_master`, `rpc_hardware_serial3_uart_master`, and `rpc_hardware_serialUSB_uart_master` interfaces.

Note that the default `Serial` debug port shares the same I/O pins as `rpc_hardware_serial_uart_master`.

## rpc_software_serial_uart_master(uint8_t rx_pin=2, uint8_t tx_pin=3, long baudrate=19200)

`rx_pin` is an interrupt capable RX pins to use to receive serial data on. `tx_pin` is any standard I/O pin. `baudrate` is the bit rate to run at. Note that baudrates above 19200 BPS may be unstable.

`SoftwareSerial` is only for AVR Arduinos like the Arduino Uno and Arduino Mega.

## Callback Buffer

This RPC library also supports creating Arduino RPC slaves so you can use the interface library for connecting Arduino's up to each other easily. However, in-order to support callback registration without dyanmic memory we have to create a callback buffer object:

openmv::rpc_callback_buffer<8> callback_buffer;

All RPC Slave Interfaces share the callback buffer space. The callback buffer stores up to X (the number in the brackets above) callbacks that are registered to a particular RPC Slave Interface. You may call `size_t buffer_size()` to get the maximum number of callbacks that will fit in the callback buffer, `size_t buffer_free()` to get how many slots are left, and `size_t buffer_used()` to get how many slots are in-use.

## RPC Slave

The interface library supports being commanded and controlled over the following interfaces:

* CAN using the `rpc_can_slave`
* I2C using the `rpc_i2c_slave`
* Hardware Serial using the `rpc_hardware_serial_uart_slave`, `rpc_hardware_serial1_uart_slave`, `rpc_hardware_serial2_uart_slave`, `rpc_hardware_serial3_uart_slave`, `rpc_hardware_serialUSB_uart_slave`
* Software Serial using the `rpc_software_serial_uart_slave`

CAN works using the MCP2515 over SPI with an Arduino or the internal CAN peripheral on the ESP32, I2C uses the default I2C bus on your Arduino, Hardware Serial uses the Hardware Serial ports on your Arduino, and Software Serial allows you to do low speed serial over any I/O pins that support interrupts on AVR based Arduinos.

_Arduino does not support a standard library for being a SPI slave. So, we do not have SPI slave support._

After creating any of the above objects you need to register callbacks using `register_callback()` and then you can call `begin()` to start things up. Finally, you can shutdown the interface with `end()`.

`register_callback()` takes the following arguments:

* A string name that the remote controller must pass to call the bound callback (can be a `char *`, `String`, or flash string `F()`). __required__
* A function pointer to a callback to be called when the remote controller requests us.

A function pointer is just the name of a function. The only requirement is that the signature function must be one of the following:

#### void rpc_callback()

This is a function that takes no arguments and returns no results.

#### void rpc_callback_with_args(void *in_data, size_t in_data_len)

This is a function which takes arguments in the form of an array of bytes.

* `in_data` is a pointer to an array of bytes being passed.
* `in_data_len` is the length of the bytes being passed.

#### void rpc_callback_returns_result(void **out_data, size_t *out_data_len)

This is a function which takes no arguments and returns data.

* `out_data` is a pointer to a pointer which should be set to the address of data you want the RPC library to return.
* `out_data_len` is a pointer to a length which should be set to the length of the data you want the RPC library to return.

#### size_t rpc_callback_returns_result_no_copy(void *out_data)

This is a function which takes no arguments and directly writes to the buffer the RPC library passed to it and returns the length of the modified data.

* `out_data` is a pointer into the scratch buffer where you can write data up to the size of the scratch buffer.

You have to then return the size of the valid written data. This callback version exists to efficently re-use the scratch buffer to return data.

#### void rpc_callback_with_args_returns_result(void *in_data, size_t in_data_len, void **out_data, size_t *out_data_len)

This is a function which takes arguments in the form of an array of bytes and returns data.

* `in_data` is a pointer to an array of bytes being passed.
* `in_data_len` is the length of the bytes being passed.
* `out_data` is a pointer to a pointer which should be set to the address of data you want the RPC library to return.
* `out_data_len` is a pointer to a length which should be set to the length of the data you want the RPC library to return.

#### size_t rpc_callback_with_args_returns_result_no_copy(void *in_data, size_t in_data_len, void *out_data)

This is a function which takes arguments in the form of an array of bytes and directly writes to the buffer the RPC library passed to it and returns the length of the modified data.

* `in_data` is a pointer to an array of bytes being passed.
* `in_data_len` is the length of the bytes being passed.
* `out_data` is a pointer into the scratch buffer where you can write data up to the size of the scratch buffer.

You have to then return the size of the valid written data. This callback version exists to efficently re-use the scratch buffer to return data.

## RPC Slave Loop

Once you have registered callbacks you just need to call `loop()` inside the Arduino `loop()` function to start processing events in the RPC library. `loop()` takes the following arguments:

* `recv_timeout` - timeout in ms to wait for a master to connect per call of `loop()` (default `100` ms)
* `send_timeout` - timeout in ms to return a result to a master (default `100` ms)

To be clear, the RPC Library `loop()` just executes once for `recv_timeout` to connect to a master and then returns (`send_timeout` is added after the master connects to return the result). So, the RPC Library `loop()` needs to be called in a loop. Normally, this method wouldn't return but on the Arduino the main Arduino `loop()` method is designed to return. So, we have to return.

_If you are using an Arduino which needs the main `loop()` to return faster than 10 Hz you may need to lower `recv_timeout` and `send_timeout`. However, keep in mind that you cannot lower `recv_timeout` and `send_timeout` too much or the RPC library will stop working correctly. Note that these timeout do not affect the length of data you can send/receive. Once data transfer starts a separate timer is used._

Finally, on successful execution of a callback `loop()` will return `true`. Otherwise `false`.

Next, below are all the constructors of the interfaces you can create. Each interface supports all the `begin()`, `register_callback()`, `loop()` and `end()` logic above.

### rpc_can_slave(int message_id=0x7FF, long bit_rate=250E3) 

`message_id` defines an 11-bit CAN message ID that all communication over CAN will happen using. `bit_rate` defines the speed the CAN bus will run at.

### rpc_i2c_slave(uint8_t slave_addr=0x12)

`slave_addr` defines the I2C address of the RPC device.

### rpc_hardware_serial_uart_slave(unsigned long baudrate=115200)

`baudrate` is the bit rate to run at. Note that depending on your Arduino you may have access to the additional `rpc_hardware_serial1_uart_slave`, `rpc_hardware_serial2_uart_slave`, `rpc_hardware_serial3_uart_slave`, and `rpc_hardware_serialUSB_uart_slave` interfaces.

Note that the default `Serial` debug port shares the same I/O pins as `rpc_hardware_serial_uart_slave`.

### rpc_software_serial_uart_slave(uint8_t rx_pin=2, uint8_t tx_pin=3, long baudrate=19200)

`rx_pin` is an interrupt capable RX pins to use to receive serial data on. `tx_pin` is any standard I/O pin. `baudrate` is the bit rate to run at. Note that baudrates above 19200 BPS may be unstable.

`SoftwareSerial` is only for AVR Arduinos like the Arduino Uno and Arduino Mega.

## Stream Mode

RPC calls take time. So much time that it's not possible to max out your interface bandwidth using them. So, the RPC library supports stream mode over each interface which will allow you to push data at the maximum possible speed your interface supports. Please see the [Arduino Stream Master](examples/arduino_to_arduino_communication_as_the_controller_device/arduino_to_arduino_communication_as_the_controller_device.ino) and [Arduino Stream Slave](examples/arduino_to_arduino_communication_as_the_remote_device/arduino_to_arduino_communication_as_the_remote_device.ino) sketches for how to use the RPC library in stream mode. Note that we do not supply examples for how to use the RPC library with the OpenMV Cam in stream mode as the OpenMV Cam will trivially overrun the data buffers on all but the most advanced Arduinos.

Streaming mode is built on top of successful RPC calls. Once you've managed to perform one you know that both the master and slave device are in sync. At this point the slave and master device can both enter stream mode at the same time to send data one direction (the RPC library is fundamentally half duplux which allows it to work over any interface).

On the sender side (which can either be the master or slave) you need to execute `bool stream_writer_setup()` and on the receiver side `bool stream_reader_setup(uint32_t queue_depth = 1)`. The `queue_depth` value passed by the reader should be a number from 1 to 255 which defines how many packets the sender is allowed to send without acknowledgement from the reader. This argument is forced to `1` on I2C/SPI interfaces which cannot handle full duplex traffic. The larger the value of `queue_depth` the more the writer and reader are decopuled from one another allowing the writer to generate packets as fast as possible without the overhead of the reader processing those packets and returning responses. That said, this argument should generally be `1` for Arduino Microcontrollers.

Both setup functions will return `true` on success and `false` on failure if a link failed to be established.

Next the writer can call `bool stream_writer_loop(rpc_stream_writer_callback_t callback, unsigned long write_timeout = 5000)` which should be called by the main Arduino `loop()`. `stream_writer_loop()` will return `true` while the link is operational and `false` once the link is torn down. The callback passed to this method should be a function of the form:

    bool rpc_stream_writer_callback(uint8_t **out_data, uint32_t *out_data_len)

* `out_data` is a pointer to a pointer which should be set to the address of data you want the RPC library to send.
* `out_data_len` is a pointer to a length which should be set to the length of the data you want the RPC library to send.

The callback should then return `true` to keep the link alive and `false` to tear it down.

_Note that stream mode bypasses the internal scratch buffer and CRC checks to move data quickly. As such, no convenience callbacks are available which provide you with a pre-allocated buffer to write to._

On the reader side `bool stream_reader_loop(rpc_stream_reader_callback_t callback, unsigned long read_timeout = 5000)` should be called to receive packets from the writer. `stream_reader_loop()` will return `true` while the link is operational and `false` once the link is torn down. The callback passed to this method should be a function of the form:

    bool rpc_stream_reader_callback_t(uint8_t *in_data, uint32_t in_data_len)

* `in_data` is a pointer to an array of bytes being passed.
* `in_data_len` is the length of the bytes being passed.

The callback should then return `true` to keep the link alive and `false` to tear it down.

_On the reader side the scratch buffer is used to receive the data so the maximum packet size is limited to the scratch buffer size._

`write_timeout` and `read_timeout` define how long in milliseconds the writer and reader should wait on all data to be transferred per packet before giving up and tearing down the link.

### Link Life

Once both devices enter stream mode and start streaming they can keep going continously until an error happens or one wants to tear down the link. Once this happens both devices must resynchronize through a successful RPC call before they can enter stream mode again.

The beauty of all of this is that you can intermix slow RPC calls with high speed streaming data transfer and jump in and out of modes trivially. RPC calls are robust against stale data in buffers generated by a torn down streaming link and can easily re-synchornize things again.

## RAW Link Access

For various reasons you may wish to send data on the underlying interface. You can do so via `get_bytes()` and `put_bytes()` which all master/slave interfaces implement. Both the master and the slave should be in sync before using these calls, then the master and slave should call `get_bytes()` and `put_bytes()` in the direction of data transfer happening with the same `size` and `timeout`.

#### bool get_bytes(uint8_t *buff, size_t size, unsigned long timeout)

* `buff` - Address of data to send.
* `size` - Size of data to send.
* `timeout` - Timeout in ms for the data to be sent.

Returns `true` if all the data was sent in the `timeout`. `false` if not.

#### bool put_bytes(uint8_t *data, size_t size, unsigned long timeout)

* `data` - Address to put received data.
* `size` - Amount of data to receive.
* `timeout` - Timeout in ms to receive the data.

Returns `true` if all the data was recevied in the `timeout`. `false` if not.
