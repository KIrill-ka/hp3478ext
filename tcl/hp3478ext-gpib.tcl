
proc gpibif_init {path addr {spd 500000} {bootloader_escape_method exit}} {
 global gpib_fd
 set ::gpib_my_addr $addr
 set ::gpib_addr_sel ""
 set ::gpib_ren 0
 set gpib_fd [open $path r+]

 if {$bootloader_escape_method eq "exit"} {
  fconfigure $gpib_fd -mode 57600,n,8,1
 } else {
  fconfigure $gpib_fd -mode 115200,n,8,1
 }
 fconfigure $gpib_fd -handshake none
 fconfigure $gpib_fd -buffering none
 fconfigure $gpib_fd -translation binary
 if {$bootloader_escape_method eq "exit"} {
  # wait for bootloader to start
  after 300
  # send bunch of invalid commands
  puts -nonewline $gpib_fd xxxxx
  fconfigure $gpib_fd -mode 115200,n,8,1
  # wait for app to initialize
  after 50
 } elseif {$bootloader_escape_method eq "wait"} {
  after 1550
 } else {
  # no bootloader or DTR is disabled/disconnected
 }
 
 fconfigure $gpib_fd -blocking 0
 # read garbage and <GPIB> prompt
 read $gpib_fd
 fconfigure $gpib_fd -blocking 1

 puts -nonewline $gpib_fd "O1\r"
 set r [gpibif_get_resp]
 if {$r eq "O1\r"} {
  set r [gpibif_get_resp]
 }
 if {$r ne "OK\r"} {
  binary scan $r cu* x
  error "echo off failed: response \"$x\""
 }
 if {$spd != 115200} {
  switch $spd {
    500000 {set s 2}
    1000000 {set s 3}
    2000000 {set s 4}
  }
  gpibif_send_cmd "OB$s"
  after 2
  fconfigure $gpib_fd -mode $spd,n,8,1
 }
 gpibif_send_cmd "OC$addr"
}

proc gpibif_get_resp {} {
 set x [gets $::gpib_fd]
 return $x
}

proc gpibif_send_cmd {c} {
 global gpib_fd
 puts -nonewline $gpib_fd "$c\r"
 set r [gpibif_get_resp]
 if {$r ne "OK\r"} { error "unexpected response $c: $r" }
}

proc gpib_bus_set {op dev_addr} {
 global gpib_my_addr gpib_addr_sel gpib_ren gpib_dev_addr
 if {$op eq "talk"} {
  set c [format "%c%c" [expr {$dev_addr+0x20}] [expr {$gpib_my_addr+0x40}]]
 } else {
  set c [format "%c%c" [expr {$dev_addr+0x40}] [expr {$gpib_my_addr+0x20}]]
 }
 if {!$gpib_ren} {
  gpibif_send_cmd "R"
  gpibif_send_cmd C$c
  set gpib_addr_sel $c
  set gpib_ren 1
 } elseif {$gpib_addr_sel ne $c} {
  gpibif_send_cmd C$c
  set gpib_addr_sel $c
 }
}

proc gpib_send {dev c} {
 gpib_bus_set talk $dev
 global gpib_fd
 set l [string length $c]
 set p 0
 set e 0
 puts -nonewline $gpib_fd "TBD\r"
 while {$l != 0} {
  if {$l > 60} {set tl 60} else {set tl $l; set e 0x80}
  incr tl -1
  set hs [string range $c $p $p+$tl]
  incr tl
  puts -nonewline $gpib_fd [binary format c [expr {$tl+$e}]]
  puts -nonewline $gpib_fd $hs
  #puts $hs
  #TODO: write next block before waiting for the result
  set res [read $gpib_fd 1]
  binary scan $res cu r
  if {$r != $tl} {
   error "write length $r < $tl @$p"
  }
  incr l -$tl
  incr p $tl
 }
 puts -nonewline $gpib_fd [binary format c 0]
}

proc gpib_recv {dev} {
 global gpib_fd
 gpib_bus_set listen $dev
 puts -nonewline $gpib_fd "TBD\r"
 set res ""
 set eoi 0
 while {1} {
  set bl [read $gpib_fd 1]
  binary scan $bl cu l
  if {$l == 0} break
  if {$l & 0x80} {
   incr l -0x80
   set eoi 1
  }
  append res [read $gpib_fd $l]
 }
 if {!$eoi && [string length $res] != 0} {
  error "no eoi after [string length $res] bytes"
 }
 return $res
}

proc gpibif_find_port {portid} {
  set devs [glob -tails -directory "/dev/serial/by-id/" *]
  foreach i $devs {
    if {$i eq $portid} {return /dev/[file tail [file readlink /dev/serial/by-id/$i]]}
  }
  return ""
}

proc gpib_read_status {} {
 #global gpib_fd
 #send_cmd C[format %c 24]
 #send_cmd CW5
 #puts $gpib_fd Z
 #set res [gets $gpib_fd]
 #send_cmd C[format %c 25]
 #return 0x[string range $res 2 3]
}

