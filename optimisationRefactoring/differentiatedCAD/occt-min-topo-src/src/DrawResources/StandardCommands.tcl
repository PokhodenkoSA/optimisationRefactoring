# Copyright (c) 1999-2014 OPEN CASCADE SAS
#
# This file is part of Open CASCADE Technology software library.
#
# This library is free software; you can redistribute it and/or modify it under
# the terms of the GNU Lesser General Public License version 2.1 as published
# by the Free Software Foundation, with special exception defined in the file
# OCCT_LGPL_EXCEPTION.txt. Consult the file LICENSE_LGPL_21.txt included in OCCT
# distribution for complete text of the license and disclaimer of any warranty.
#
# Alternatively, this file may be used under the terms of Open CASCADE
# commercial license or contractual agreement.

#
# Draw standard initialisation
#

#################################################
# prompts
#################################################

set Draw_CmdIndex 0
set tcl_prompt1 {
    incr Draw_CmdIndex
    puts -nonewline "Draw\[$Draw_CmdIndex\]> "
}

set tcl_prompt2 {puts -nonewline "> "}


#################################################
# the help command in TCL
#################################################


proc help {{command ""} {helpstring ""} {group "Procedures"}} {

    global Draw_Helps Draw_Groups

    if {$command == ""} {

    # help general
    foreach h [lsort [array names Draw_Groups]] {
        puts ""
        puts ""
        puts $h
        set i 0
        foreach f [lsort $Draw_Groups($h)] {
        if {$i == 0} {
            puts ""
            puts -nonewline "  "
        }
        puts -nonewline $f
        for {set j [string length $f]} {$j < 15} {incr j} {
            puts -nonewline " "
        }
        incr i
        if {$i == 4} {set i 0}
        }
        puts ""
    }
    } elseif {$helpstring == ""} {

    # help function
    set isfound 0
    foreach f [lsort [array names Draw_Helps]] {
        if {[string match $command $f]} {
        puts -nonewline $f
        for {set j [string length $f]} {$j < 15} {incr j} {
            puts -nonewline " "
        }
        puts " : $Draw_Helps($f)"
        set isfound 1
        }
    }
    if {!$isfound} {
        if {[string first * $command] != -1} {
            puts "No matching commands found!"
        } else {
            puts "No help found for '$command'! Please try 'help $command*' to find matching commands."
        }
    }
    } else {

    # set help
    lappend Draw_Groups($group) $command
    set Draw_Helps($command) $helpstring
    }
    
    flush stdout
}

help help {help pattern, or help command string group, to set help} {DRAW General Commands}
#################################################
# the getsourcefile command in TCL
#################################################


proc getsourcefile {{command ""}} {

    global Draw_Helps Draw_Groups Draw_Files

    set out {}
    if {$command == ""} {

	# help general
	foreach h [lsort [array names Draw_Groups]] {
	    lappend out "" "" "$h"
	    set i 0
	    foreach f [lsort $Draw_Groups($h)] {
		if {$i == 0} {
		    lappend out ""
		}
		incr i
#
# check that the command has its source file set
#
		foreach command_that_has_file [array names Draw_Files] {
		    if {($command_that_has_file == $f)} {
			lappend out [format {%-20s %s} $f $Draw_Files($f)]
		    }
		}
	    }
	}
    } else {

	# getsourcefile fonction
	append command "*"
	foreach f [lsort [array names Draw_Files]] {
	    if {[string match $command $f]} {
                lappend out [format {%-20s %s} $f $Draw_Files($f)]
	    }
	}
	
    } 
    return [join $out "\n"]
}

help getsourcefile {getsourcefile, or getsourcefile command } {DRAW General Commands}

#################################################
# whatis
#################################################

#proc gwhatis {aVarName} {
#    global $aVarName
#    puts -nonewline $aVarName; puts -nonewline " is a "; puts [dtyp ${aVarName}]
#}

proc whatis args {
    set __out_string ""
    foreach i $args {
	if {$i == "."} {set i [dname $i]}
	#gwhatis $i
	global $i
	set __tmp_string "$i is a [dtyp $i]\n"
	set __out_string "${__out_string}${__tmp_string}"
    }
    return ${__out_string}
}

help whatis "whatis object1 object2 ..." 

#################################################
# library, lsource
#################################################

proc library lib {
    global auto_path
    set auto_path [linsert $auto_path 0 $lib]
    if [file readable $lib/LibraryInit] {
	puts "Loading $lib/LibraryInit"
	uplevel "source $lib/LibraryInit"
    }
}

proc lsource file {
    if [file readable $file] {source $file} else {
	global auto_path
	foreach dir $auto_path {
	    if [file readable $dir/$file] {
		uplevel #0 "source $dir/$file"
		break
	    }
	}
    }
}

#################################################
# directory
#################################################

proc isgdraw {var} {
    global $var
    return [isdraw $var]
}

proc directory {{joker *}} {
    set res ""
    foreach var [info globals $joker] { 
	if [isgdraw $var] {lappend res $var}
    }
    return $res
}

help directory {directory [pattern], list draw variables} {DRAW Variables management}

proc lsd {} { exec ls [datadir] }

proc dall {} {
    set schmurtz ""
    foreach var [info globals] { 
	global $var
	if [isdraw $var] {
	    if ![isprot $var] {
		lappend schmurtz $var; unset $var
	    }
	}
    }
    return $schmurtz
}

#################################################
# repeat, do
#################################################

proc repeat {val script} {
    for {set i 1} {$i <= $val} {incr i} {uplevel $script}
}

proc do {var start end args} {
    global errorInfo errorCode
    if {[llength args] == 1} {
	set incr 1
	set body args
    } else {
	set incr [lindex 1 args]
	set body [lindex 2 args]
    }
    upvar $var v
    if {[dval $incr] < 0} {set rel >=} else {set rel <=}
    for {dset v $start} {[dval v] $rel [dval end]} {dset v [dval v+($incr)]} {
	set code [catch {uplevel $body} string]
	if {$code == 1} {
	    return -code error -errorInfo $errorInfo -errorcode $errorCode $string
	} elseif {$code == 2} {
	    return -code return $string
	}elseif {$code == 3} {
	    return
	} elseif {$code > 4} {
	    return -code $code $string
	}
    }
}

#################################################
# datadir, save, restore
#################################################

set Draw_DataDir "."

proc datadir {{dir ""}} {
    global Draw_DataDir
    if {$dir != ""} {
	if {![file isdirectory $dir]} {
	    error "datadir : $dir is not a directory"
	} else {
	    set Draw_DataDir $dir
	}
    }
    return $Draw_DataDir
}

help datadir {datadir [directory]} "DRAW Variables management"

proc save {name {file ""}} {
    if {$file == ""} {set file $name}
    upvar $name n
    if {![isdraw n]} {error "save : $name is not a Draw variable"}
    global Draw_DataDir
    bsave n [file join $Draw_DataDir $file]
    return [file join $Draw_DataDir $file]
}

help save {save variable [filename]} "DRAW Variables management"

proc restore {file {name ""}} {
    if {$name == ""} {
        # if name is not given explicitly, use name of the file w/o extension
        set name [file rootname [file tail $file]]
    }
    global Draw_DataDir
    upvar $name n
    brestore [file join $Draw_DataDir $file ] n
    return $name
}

help restore {restore filename [variablename]} "DRAW Variables management"

#################################################
# misc...
#################################################

proc ppcurve {a} {
	2dclear;
	uplevel pcurve $a;
	2dfit;
}

#################################################
# display and donly with jokers
#################################################


proc disp { args } {
    set res ""
    foreach joker $args {
	if { $joker == "." } {
             dtyp .
             set joker [lastrep id x y b]
	}
        foreach var [info globals $joker] { 
	   if { $var == "." } {
               dtyp .
               set var [lastrep id x y b]
	   }
	   if [isgdraw $var] {lappend res $var}
        }
    }
    uplevel #0 eval display $res
    return $res
}


proc donl { args } {
    set res ""
    foreach joker $args {
	if { $joker == "." } {
             dtyp .
             set joker [lastrep id x y b]
	}
        foreach var [info globals $joker] { 
	   if { $var == "." } {
               dtyp .
               set var [lastrep id x y b]
	   }
	   if [isgdraw $var] {lappend res $var}
        }
    }
    uplevel #0 eval donly $res
    return $res
}

proc don { args } {
    set res ""
    foreach joker $args {
	if { $joker == "." } {
             dtyp .
             set joker [lastrep id x y b]
	}
        foreach var [info globals $joker] { 
	   if { $var == "." } {
               dtyp .
               set var [lastrep id x y b]
	   }
	   if [isgdraw $var] {lappend res $var}
        }
    }
    uplevel #0 eval donly $res
    return $res
}
