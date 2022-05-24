# Veggie_PID
Simple project to automate a low pressure filter system for waste vegetable oil (WVO) used to power diesel vehicles.  
  
The project controller uses an arduino nano with an attached pressure sensor (30 PSI) and a small 1" oled screen.  
  
The outputs of the controller are to a mosfet which drives a bildge pump (boat) for pumping oil. The power for the uc (microcontroller) and pump is provided with a battery, or in this case a computer power supply.  
  
The heater is controlled via a ssr (solid state relay) which drives a water heater element within the sock. Powered by mains voltage. (care is required!)  
**Please understand the risk of mains power and vegetable oil**  

The sock (filter) is from mcmaster. part number: 5162K117 (5 micron)  
Mcmaster does not sell a 8in adapter, so one was 3d printed out of ASA  
The bag adaptor flange is 1 1/4" NTP  
The bag is held on with the aid of a large (11" in this case) metal clamp.  

![terminal block enclosure](https://user-images.githubusercontent.com/592299/170122953-d74acfb1-226f-4707-9684-024d3d3db979.PNG)

![filter bag adaptor](https://user-images.githubusercontent.com/592299/170123097-ad1e23d4-b19e-4c54-a45d-796858d49144.PNG)

![IMG_20190506_134606](https://user-images.githubusercontent.com/592299/170127929-1c71fe27-3fbc-488b-bab3-a898713480f0.jpg)
