
proc rfegen_open {dev} {
 global rfegen_fd
 set rfegen_fd [open $dev r+]
 fconfigure $rfegen_fd -mode 500000,n,8,1
 fconfigure $rfegen_fd -handshake none
 fconfigure $rfegen_fd -buffering none
 fconfigure $rfegen_fd -translation binary
}

proc rfegen_cw {freq_khz pwr} {
 global rfegen_fd
 if {$pwr > 3} {
   set hpwr 1
   incr pwr -3
 } else {
   set hpwr 0
 }

 set cmd [binary format "aca*a7aaaa" # 18 C3-F: [format %07d $freq_khz] , $hpwr , $pwr]
 puts -nonewline $rfegen_fd $cmd
}

proc rfegen_track_start {start_freq_khz n_steps step_khz pwr} {
 global rfegen_fd
 if {$pwr > 3} {
   set hpwr 1
   incr pwr -3
 } else {
   set hpwr 0
 }

 set cmd [binary format "aca*a7aaaaaa4aa7" # 31 C3-T: [format %07d $start_freq_khz] , $hpwr , $pwr , [format %04d $n_steps] , [format %07d $step_khz]]
 puts -nonewline $rfegen_fd $cmd
}

proc rfegen_track_step {step_n} {
 global rfegen_fd
 set cmd [binary format "aca*S" # 5 k $step_n]
 puts -nonewline $rfegen_fd $cmd
}

proc rfegen_off {} {
 global rfegen_fd
 set cmd [binary format "aca*" # 5 CP0]
 puts -nonewline $rfegen_fd $cmd
}
