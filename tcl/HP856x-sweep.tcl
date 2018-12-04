#!/usr/bin/tclsh8.6

proc freq_to_khz {s} {
  set s [string map -nocase {khz "" mhz "000" ghz "000000"} $s]
  return $s
}

set npts 100
set method z
set power 0
set norm 0
set converter_addr 21
set hp_addr 18

for {set i 0} {$i < [llength $argv]} {incr i} {
 switch -- [lindex $argv $i] {
  -analyser_address - -aa {
       incr i; set hp_addr [lindex $argv $i] }
  -generator_address - -ga {
       incr i; set converter_addr [lindex $argv $i] }
  -analyser - -a {
       incr i; set hp_port [lindex $argv $i] }
  -generator - -g {
       incr i; set rfe_port [lindex $argv $i] }
  -freq - -frequency - -f {
       incr i
       set s [string map {- " "} [lindex $argv $i]]
       set start_freq [freq_to_khz [lindex $s 0]]
       set end_freq [freq_to_khz [lindex $s 1]]}
  -method - -meth - -m {
       incr i
       set s 
       switch [lindex $argv $i] {
         z - zerospan {set method z}
         p - peak - peaksearch  {set method p}
       }
  }
  -norm - -normalize - -n {set norm 1}
  -points - -pts - -p {
       incr i
       set npts [lindex $argv $i] 
  }
  -bw - -bandwidth - -b {
       incr i
       set bw [freq_to_khz [lindex $argv $i]]
  }
  -level - -power - -l {
       incr i
       set power [lindex $argv $i]
  }
 }
}

set step_size [expr {($end_freq-$start_freq)/($npts-1)}]

if {![info exists bw]} {
 if {$step_size > 20} {
  set bw 30
 } else {
  set bw 10
 }
}

if {![info exists hp_port] || ![info exists rfe_port]} {
  set devs [glob -tails -directory "/dev/serial/by-id/" *]
  foreach i $devs {
   switch $i {
     usb-FTDI_FT232R_USB_UART_A98B7D1P-if00-port0 {
       set hp_port /dev/[file tail [file readlink /dev/serial/by-id/$i]]
     }
     usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 {
       set rfe_port /dev/[file tail [file readlink /dev/serial/by-id/$i]]
     }
   }
  }
}

source hp3478ext-gpib.tcl
source rfesiggen.tcl

adapter_init $hp_port $converter_addr

rfegen_open $rfe_port
gpib_send $hp_addr "NORMLIZE OFF"
gpib_send $hp_addr "CLRW TRA"
gpib_send $hp_addr "TDF B"

if {$method eq "z"} {
 gpib_send $hp_addr "SNGLS;CF ${start_freq}KHZ;SP ZERO;ST 30MS"
 gpib_send $hp_addr "SS ${step_size}KHZ"
 gpib_send $hp_addr "MKT 15MS"
 gpib_send $hp_addr "RB ${bw}KHZ"
} else {
 gpib_send $hp_addr "SNGLS"
 gpib_send $hp_addr "FA ${start_freq}KHZ"
 gpib_send $hp_addr "FB [expr {$start_freq+($npts-1)*$step_size}]KHZ"
}

rfegen_track_start $start_freq $npts $step_size $power
puts "step: $step_size"
after 500
set res ""
for {set s 1} {$s <= $npts} {incr s} {

 gpib_send $hp_addr "TS;DONE?"
 while {[gpib_recv $hp_addr] != 1} { } 

 rfegen_track_step $s
 if {$method ne "z"} { gpib_send $hp_addr "MKPK" }
 gpib_send $hp_addr "MKA?"
 set val [gpib_recv $hp_addr]
 set val [expr {$val}]
 lappend res $val
 puts "[expr {$s-1}]: $val"
 if {$method eq "z"} { gpib_send $hp_addr "CF UP" }
}
rfegen_off

set res1 ""
for {set i 0} {$i <= 600} {incr i} {
 append res1 [format %0.2f [lindex $res [expr {$i*$npts/601}]]]
 if {$i != 600} {append res1 ","}
}

#puts $res1
gpib_send $hp_addr "FA ${start_freq}KHZ"
gpib_send $hp_addr "FB [expr {$start_freq+($npts-1)*$step_size}]KHZ"
gpib_send $hp_addr "TDF P"

gpib_send $hp_addr "FA ${start_freq}KHZ"
if {$norm} {
 # the following calls to STORETHRU/TS are only needed to
 # to avoid BAD NORM error
 gpib_send $hp_addr "STORETHRU"
 # although it dislays "THRU STORED" immediately
 # if we don't TS, it would wipe TRACE B afterwards
 gpib_send $hp_addr "TS;DONE?"
 while {[gpib_recv $hp_addr] != 1} { } 

 # write our normalization trace to B
 gpib_send $hp_addr "VIEW TRB"
 gpib_send $hp_addr "TRB $res1"
 gpib_send $hp_addr "DONE?"
 while {[gpib_recv $hp_addr] != 1} { } 
 # SAVES 9 was an attempt to save the normalization state
 # in alternative to STORETHRU
 #gpib_send $hp_addr "SAVES 9"

 #gpib_send $hp_addr "TRB?"
 #puts [gpib_recv $hp_addr]
 gpib_send $hp_addr "CLRW TRA"
 gpib_send $hp_addr "CONTS"
} else {
 gpib_send $hp_addr "VIEW TRA"
 gpib_send $hp_addr "TRA $res1"
 gpib_send $hp_addr "DONE?"
 while {[gpib_recv $hp_addr] != 1} { } 
#gpib_send $hp_addr "TRA?"
#puts [gpib_recv $hp_addr]
 gpib_send $hp_addr "CONTS"
}

if {$norm} {
#gpib_send $hp_addr "STORETHRU"
#gpib_send $hp_addr "CONTS"
} else {
 gpib_send $hp_addr "NORMLIZE ON"
}

adapter_send_cmd L

