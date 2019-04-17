# forgotten_dreams
this is a fork of qsxcv's and ninox's bst pmw3360 libre mouse firmware. this is gpl3 licensed, the originals are MIT.
It is not mentioned in the instructions in the youtube video but using optional 5k pullup resistors on each pin and 10k
 on both pins of the wheel encoder will extend switch life. 
 
I created this fork because I wanted to experiment with 3 pin switches. Later on I added in ben buxtons fault tolerant encoder code. It is a work in progress so if you want reliability, grab the original firmware,
as there are so many features is is difficult to test everything. 
If you want to get in contact send a pm to gipetto in overclock.net forums.

http://www.youtube.com/watch?v=nyb6M89QrWI

https://www.overclock.net/forum/375-mice/1561041-reverse-engineering-3366-a-35.html

features:

software debouncing with variable debounce period

hardware debouncing, i.e no superfluous clicks, or unclicks, no debounce period so possible to do ultra fast burst fire.

ignores illegal encoder gray codes,

ignores wheel scrolls when middle click is held,

can use a mix of software debounced 2 pin switches and 3 pin,

x and y axis reversal,

polling rate from 1000khz to 62.5khz and lower,

rgb support, (untested)

dpi button, (untested)

3.3v mcu support (only atmega32u4),

variable lift off distance (LOD) 1-3 mm and disable

disable and enable sleep mode

things it can't do.

can't bind the same key twice without minor mods( could be added at some stage)

can't emulate a keyboard, have to use os software for that.

only works reliably with mechanical encoders, ec10 or ec11. I used optical for a while but the detectors died from overcurrent.

can't software debounce random pins. Pins must be ordered from D0 to D5. L,R,M,B,F,dpi
