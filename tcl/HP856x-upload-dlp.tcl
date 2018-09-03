#!/usr/bin/tclsh8.6

set my_addr 21
set hp_addr 18

for {set i 0} {$i < [llength $argv]-1} {incr i} {
 switch -- [lindex $argv $i] {
  -analyser_address - -aa {
       incr i; set hp_addr [lindex $argv $i] }
  -analyser - -a {
       incr i; set hp_port [lindex $argv $i] }
 }
}

set f [lindex $argv $i]

source hp3478ext-gpib.tcl

if {![info exists hp_port]} {
 set hp_port [gpibif_find_port "usb-FTDI_FT232R_USB_UART_A98B7D1P-if00-port0"]
}

gpibif_init $hp_port $my_addr

set dlp ""
set ff [open $f r]
while {![eof $ff]} {
 set s [gets $ff]
 set s [regsub ";;.*$" $s ""]
 set s [string trim $s " \t"]
 if {$s eq ""} continue
 append dlp $s
 append dlp "\n"
}
#puts -nonewline $dlp

gpib_send $hp_addr $dlp
gpibif_send_cmd L
