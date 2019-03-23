#!/usr/bin/tclsh8.6

source ihex.tcl

set list 0

set arg 0
if {[lindex $argv $arg] eq "-d"} {
  incr arg
  set def [lindex $argv $arg]
  incr arg
} elseif {[lindex $argv $arg] eq "-l"} {
  set list 1
  incr arg
}
set out [lindex $argv $arg]
incr arg

proc def2name {def} {
 return [regsub {DEF[0-9]+_(.*)} $def \\1]
}
proc eep_set_var {eep_var name value} {
 upvar $eep_var eep
 global opt_sz
 set addr $::opt($name)

 if {$opt_sz($name) == 1} {
  set eep [binary format a*@${addr}c $eep $value]
 } else {
  set eep [binary format a*@${addr}s $eep $value]
 }
}

proc eep_get_var {eep name} {
 global opt_sz opt
 set addr $opt($name)
 if {$opt_sz($name) == 1} {
  binary scan $eep @${addr}cu old_value
 } else {
  binary scan $eep @${addr}su old_value
 }
 if {![info exists old_value]} {return <undef>}
 return $old_value
}

set f [open eepmap.h r]
while {![eof $f]} {
  set s [gets $f]
  set var [lindex $s 1]
  if {[string first EEP_ADDR_ $var] == 0} {
   set n [string range $var 9 end]
   set opt($n) [lindex $s 2]
   if {![info exists opt_sz($n)]} {
     set opt_sz($n) 1
   }
  } elseif {[string first EEP_DEF $var] == 0} {
    set opt_def([string range $var 4 end]) [lindex $s 2]
  } elseif {[string first EEP_SIZE_ $var] == 0} {
    set n [string range $var 9 end]
    set opt_sz($n) [lindex $s 2]
  }
}
close $f

if {[info exists def]} {
  set eep {}
  foreach v [array names opt_def DEF0*] {
    eep_set_var eep [def2name $v] $opt_def($v)
  }
  foreach v [array names opt_def $def*] {
    eep_set_var eep [def2name $v] $opt_def($v)
  }
# ihex_write $out [list 0 $eep]
} else {

 set eep_chunks [ihex_read $out]
 if {[llength $eep_chunks] != 2} {
   error "discontiguous hex file"
 }
 set eep [lindex $eep_chunks 1]
}

if {$list} {
 foreach n [lsort [array names opt]] {
  puts "$n=[eep_get_var $eep $n] @$opt($n)/$opt_sz($n) "
 }
 exit 0
}

foreach {name value} [lrange $argv $arg end] {
 set old_value [eep_get_var $eep $name]
 set addr $opt($name)
 if {$opt_sz($name) == 1} {
  set eep [binary format a*@${addr}c $eep $value]
 } else {
  set eep [binary format a*@${addr}s $eep $value]
 }
 puts "$name: $old_value->$value"
}

ihex_write $out [list 0 $eep]
