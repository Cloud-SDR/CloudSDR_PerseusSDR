# Cloud-SDR-PerseusSDR
Open source driver to add PerseusSDR HF receiver to [Cloud-SDR SDRNode](http://www.cloud-sdr.com/). 

SDRNode can load custom driver files from shared library.
to implement you own driver, you have to code all the functions defined in external_hardware_def.h and then update your customhardware.js file.

Example :
```javascript
// Load XXX driver shared library
if( SDRNode.loadDriver('CloudSDR_XXXX','') ) {
	print('XXX Driver loaded');
}
```

Original project gets compiled to CloudSDR_Perseus.DLL for Windows or CloudSDR_Perseus.so for Linux.
your Javascript call should then be:
```javascript
// load the PerseusSDR bridge
if( SDRNode.loadDriver('CloudSDR_Perseus','') ) {
	print('AirSpy Driver loaded');
}
```
No option implemented yet (second paramater for function loadDriver() should be left as empty string)

The key steps are :
* when the javascript call is handled, SDRNode calls *initLibrary()*.
* then SDRNode calls *getBoardCount()*
* SDRNode assumes this first call, if *getBoardCount()* >= 1, that we have one hardware device of this type
* SDRNode loops from 0 to *getBoardCount()-1* to retrieve parameters for other devices, BUT DOES NOT CALL initLibrary()


# Building
Using Qt Creator just open the .pro file and compile (release). The binary file will be copied to \SDRNode\addons subfolder.
# windows
Warning with the GCC compile option in this project. For C files, you really need 
QMAKE_CFLAGS =  -mno-ms-bitfields  

Custom drivers can be loaded at any time by scripting. 
Check http://wiki.cloud-sdr.com/doku.php?id=documentation for more details.
