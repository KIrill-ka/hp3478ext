
proc rfegen_open {dev} {
 global rfegen_fd
 set rfegen_fd [open $dev r+]
 fconfigure $rfegen_fd -mode 500000,n,8,1
 fconfigure $rfegen_fd -handshake none
 fconfigure $rfegen_fd -buffering none
 fconfigure $rfegen_fd -translation binary
}

proc rfegen_cvt_pwr {pwr_lev hpwr_var pwr_var} {
 upvar $hpwr_var hpwr $pwr_var pwr

 set pwr $pwr_lev
 if {$pwr > 3} {
   set hpwr 1
   incr pwr -4
 } else {
   set hpwr 0
 }
}

proc rfegen_cw {freq_khz pwr} {
 global rfegen_fd

 rfegen_cvt_pwr $pwr hpwr pwr

 set cmd [binary format "aca*a7aaaa" # 18 C3-F: [format %07d $freq_khz] , $hpwr , $pwr]
 puts -nonewline $rfegen_fd $cmd
}

proc rfegen_track_start {start_freq_khz n_steps step_khz pwr} {
 global rfegen_fd

 rfegen_cvt_pwr $pwr hpwr pwr

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

proc rfegen_send_cmd {c} {
 global rfegen_fd
 set l [string length $c]
 incr l 2
 puts -nonewline $rfegen_fd [binary format "aca*" # $l $c]
}

proc rfegen_cw_exp {freq_khz pwr} {
 # FW 1.31: C5-F command does accept %+06.2f, but it rounds to .25 anyway.
 #          The tracking mode doesn't accept +XX.XX format for some reason,
 #          so we use +XX.X
 rfegen_send_cmd C5-F:[format %07d $freq_khz],[format %+05.1f $pwr]
}

proc rfegen_track_exp_start {start_freq_khz n_steps step_khz pwr} {
 rfegen_send_cmd C5-T:[format %07d $start_freq_khz],[format %+05.1f $pwr],[format %04d $n_steps],[format %07d $step_khz]
}
