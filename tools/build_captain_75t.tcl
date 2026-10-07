# Captain 75T bitstream build for the bt repo. Run from anywhere:
#   vivado -mode batch -source tools/build_captain_75t.tcl -notrace
set repo_dir [file normalize [file join [file dirname [info script]] ..]]
set proj [file join $repo_dir pcileech_enigma_x1 pcileech_enigma_x1.xpr]
if {![file exists $proj]} {
    puts "ERROR: project not found: $proj"
    exit 1
}

cd $repo_dir
open_project $proj
set_param general.maxThreads 8
catch {set_property generic {PRODUCTION=1} [get_filesets sources_1]}

set patch [file join $repo_dir tools pcie_core_patch.tcl]
if {[file exists $patch]} {
    set ::origin_dir $repo_dir
    source $patch
}

# Re-sync imported sources with the repo: import_files copies go stale and the
# run reset does not re-import them. Skip this and the bitstream silently
# contains old RTL.
puts "RE-SYNC imported sources with repo ..."
set _syncbase [file join $repo_dir pcileech_enigma_x1 pcileech_enigma_x1.srcs sources_1 imports [file tail $repo_dir]]
set _synced 0
foreach f [concat [glob -nocomplain [file join $repo_dir src *.sv]] [glob -nocomplain [file join $repo_dir src *.svh]]] {
    set dst [file join $_syncbase src [file tail $f]]
    if { [file exists $dst] } {
        file copy -force $f $dst
        puts "  re-synced src/[file tail $f]"
        incr _synced
    }
}
foreach f [glob -nocomplain [file join $repo_dir pcie_7x *.v]] {
    set dst [file join $_syncbase pcie_7x [file tail $f]]
    if { [file exists $dst] } {
        file copy -force $f $dst
        puts "  re-synced pcie_7x/[file tail $f]"
        incr _synced
    }
}
# import_files creates snapshots: explicitly refresh all project XDC copies.
foreach imported [get_files -quiet -of_objects [get_filesets constrs_1] *.xdc] {
    set pristine [file join $repo_dir src [file tail $imported]]
    if {[file exists $pristine]} {
        file copy -force $pristine $imported
        puts "  re-synced constraint [file tail $imported]"
    }
}
puts "  re-sync done ($_synced file(s) updated)"

foreach ip_file [get_files -quiet -of_objects [get_filesets sources_1] *.xci] {
    catch {set_property IS_LOCKED 0 $ip_file}
    catch {set_property synth_checkpoint_mode None $ip_file}
}

set_property STEPS.WRITE_BITSTREAM.ARGS.BIN_FILE 1 [get_runs impl_1]
# Do not guide a new constrained design with stale, unconstrained synth checkpoints.
set_property AUTO_INCREMENTAL_CHECKPOINT 0 [get_runs synth_1]
set_property INCREMENTAL_CHECKPOINT {} [get_runs synth_1]
set_property AUTO_INCREMENTAL_CHECKPOINT 0 [get_runs impl_1]
set_property INCREMENTAL_CHECKPOINT {} [get_runs impl_1]
reset_run synth_1
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} { error "Implementation did not complete" }
open_run impl_1
if {[llength [get_clocks -quiet pcie_user_clk]] != 1} { error "PCIe user clock is missing" }
report_timing_summary -file [file join $repo_dir network_timing.rpt]
report_utilization -file [file join $repo_dir network_utilization.rpt]
set bad_paths [get_timing_paths -quiet -slack_lesser_than 0 -max_paths 1]
if {[llength $bad_paths] != 0} { error "Timing failed; do not deliver this bitstream" }
set bad_hold [get_timing_paths -quiet -delay_type min -slack_lesser_than 0 -max_paths 1]
if {[llength $bad_hold] != 0} { error "Hold timing failed; do not deliver this bitstream" }



set impl_status [get_property STATUS [get_runs impl_1]]
set impl_progress [get_property PROGRESS [get_runs impl_1]]
puts "impl_1 STATUS=$impl_status PROGRESS=$impl_progress"

set impl_dir [file join $repo_dir pcileech_enigma_x1 pcileech_enigma_x1.runs impl_1]
set bit_file [file join $impl_dir pcileech_enigma_x1_top.bit]
set bin_file [file join $impl_dir pcileech_enigma_x1_top.bin]
set routed_dcp [file join $impl_dir pcileech_enigma_x1_top_routed.dcp]
if {![file exists $bit_file] && [file exists $routed_dcp]} {
    puts "bit missing; export from routed.dcp"
    open_checkpoint $routed_dcp
    write_bitstream -force -bin_file $bit_file
}
if {![file exists $bit_file]} {
    puts "ERROR: bitstream not written: $bit_file"
    exit 1
}
if {![file exists $bin_file]} {
    set newest ""
    set newest_t 0
    foreach f [glob -nocomplain [file join $impl_dir *.bin]] {
        if {[file mtime $f] >= $newest_t} {
            set newest $f
            set newest_t [file mtime $f]
        }
    }
    if {$newest ne ""} {
        set bin_file $newest
        puts "using newest impl bin: $bin_file"
    } elseif {[file exists $routed_dcp]} {
        puts "bin missing; export from routed.dcp"
        if {[current_design -quiet] eq ""} {
            open_checkpoint $routed_dcp
        }
        write_bitstream -force -bin_file $bin_file
    }
}
if {![file exists $bin_file]} {
    puts "ERROR: bin not written"
    exit 1
}
file copy -force $bin_file [file join $repo_dir pcileech_captain_75t.bin]
file copy -force $bit_file [file join $repo_dir pcileech_captain_75t.bit]
puts "OK -> pcileech_captain_75t.bit size=[file size [file join $repo_dir pcileech_captain_75t.bit]]"
puts "OK -> pcileech_captain_75t.bin size=[file size [file join $repo_dir pcileech_captain_75t.bin]]"
exit 0
