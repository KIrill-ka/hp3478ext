#!/usr/bin/tclsh8.6

set elf [lindex $argv 0]
set out [lindex $argv 1]
set f [open "|avr-objdump -j .eeprom -t $elf"]
while {![eof $f]} {
 set s [gets $f]
 set t [lindex $s 2]
 if {$t ne "O"} continue
 set name [lindex $s end]
 set opt($name) 0x[lindex $s 0]
 set opt_sz($name) 0x[lindex $s end-1]
 set opt($name) [expr {$opt($name)-0x810000}]
 set opt_sz($name) [expr {$opt_sz($name)}]
}
close $f


set f [file tempfile tmp]
close $f

exec avr-objcopy -j .eeprom --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0 -O binary $elf $tmp
set f [open $tmp r]
fconfigure $f -translation binary
set eep [read $f]
close $f

proc get_val {eep name} {
 global opt_sz opt
 set addr $opt($name)
 if {$opt_sz($name) == 1} {
  binary scan $eep @${addr}cu old_value
 } else {
  binary scan $eep @${addr}su old_value
 }
 return $old_value
}

if {$out eq ""} {
 foreach n [array names opt] {
  puts "$n=[get_val $eep $n] @$opt($n)/$opt_sz($n) "
 }
 file delete $tmp
 exit 0
}

foreach {name value} [lrange $argv 2 end] {
 set old_value [get_val $eep $name]
 set addr $opt($name)
 if {$opt_sz($name) == 1} {
  set eep [binary format a*@${addr}c $eep $value]
 } else {
  set eep [binary format a*@${addr}s $eep $value]
 }
 puts "$name: $old_value->$value"
}

set f [open $tmp w]
fconfigure $f -translation binary
puts -nonewline $f $eep
close $f

exec avr-objcopy -I binary -O ihex $tmp $out
file delete $tmp
