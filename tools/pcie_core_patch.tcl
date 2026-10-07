# T11: freeze BCM5751 PCIe identity after project generate / IP regen.
#
# Decision: zdma/ is a different generation (x4 Gen2 128-bit, v3.3.20), not a
# parent of the active x1 Gen1 64-bit GTP core. A line-by-line zdma→current
# transform is not maintainable. Source of truth is repo pcie_7x/ (four files).
#
# Flow:
#   1. Surgical, idempotent parameter fixes (GUI-unexposed + regen residue)
#   2. Overlay the four repo files onto any Vivado import snapshot
#   3. Verify identity
#
# Do not touch pcie_7x/zdma/. Do not rewrite CORE_GENERATION_INFO.
#
# Inventory (git diff --no-index zdma vs active, CORE_GENERATION_INFO omitted):
#
# pcie_7x_0_core_top.v  (zdma already had 14E4:1677 / BAR0=FFFF0004 / rev 11)
#   [2026-08-22 回滚] ASPM 延迟值保持 6/6（T01 曾改 7/4/3，上板开机黑屏，
#   判定 GTP 达不到广告的退出时序；INTERRUPT_PIN 回 0，见 FIDELITY_PLAN §5 修复记录）
#   DEV_CAP_ENDPOINT_L0S_LATENCY          6 (保持)
#   LINK_CAP_L0S_EXIT_LATENCY_*           6 (保持)
#   LINK_CAP_L1_EXIT_LATENCY_*            6 (保持)
#   EXT_CFG_CAP_PTR                       6'h3F → 6'h1A
#   PM_CAP_NEXTPTR                        8'h50 → 8'h58
#   CMD_INTX_IMPLEMENTED                  TRUE → FALSE
#   LINK_CAP_MAX_LINK_SPEED               4'h2 → 4'h1
#   LINK_CAP_MAX_LINK_WIDTH               6'h4 → 6'h1
#   LINK_CTRL2_TARGET_LINK_SPEED          4'h2 → 4'h0
#   PIPE_PIPELINE_STAGES                  1 → 0
#   C_DATA_WIDTH                          128 → 64
#   USER_CLK_FREQ                         3 → 1
#   USER_CLK2_DIV2                        TRUE → FALSE
#   TRN_DW                                TRUE → FALSE
#   DISABLE_LANE_REVERSAL                 FALSE → TRUE
#   PCIE_GT_DEVICE                        (GTX-class) → GTP
#   plus capability-block reorder / MSIX offset wiring
#
# pcie_7x_0_pcie2_top.v
#   c_pm_cap_next_ptr                     48 → 58
#   core_top instance overrides added:
#     CMD_INTX_IMPLEMENTED FALSE
#     INTERRUPT_PIN 8'h0   (2026-08-22 回滚：曾 8'h1，引导期黑屏)
#     EXT_CFG_CAP_PTR 6'h1A
#     MSI_CAP_MULTIMSGCAP 3
#     MSI_CAP_PER_VECTOR_MASKING_CAPABLE FALSE
#
# pcie_7x_0_pcie_top.v / pcie_7x_0_pcie_7x.v
#   PM_CAP_NEXTPTR default                8'h48 → 8'h58
#   pcie_7x.v TRNTD/TRNTREM padded for 64-bit datapath
#
# GUI-expressible knobs live in ip/pcie_7x_0.xci (this script does not edit XCI).

namespace eval t11 {
  variable repo ""
  variable files {
    pcie_7x_0_core_top.v
    pcie_7x_0_pcie2_top.v
    pcie_7x_0_pcie_top.v
    pcie_7x_0_pcie_7x.v
  }
}

proc t11_repo {} {
  if {$::t11::repo ne ""} {
    return $::t11::repo
  }
  set here [file dirname [file normalize [info script]]]
  set cand [file normalize [file join $here ..]]
  if {[info exists ::origin_dir] && [file isdirectory [file join $::origin_dir pcie_7x]]} {
    set cand [file normalize $::origin_dir]
  }
  set ::t11::repo $cand
  return $::t11::repo
}

proc t11_is_zdma {path} {
  set n [string map {\\ /} $path]
  return [expr {[string match "*/zdma/*" $n] || [string match "*/zdma" $n]}]
}

proc t11_read {path} {
  set fd [open $path r]
  fconfigure $fd -translation auto -encoding utf-8
  set data [read $fd]
  close $fd
  return $data
}

proc t11_write {path data} {
  set fd [open $path w]
  fconfigure $fd -translation lf -encoding utf-8
  puts -nonewline $fd $data
  close $fd
}

# Replace `parameter ... NAME = OLD` keeping the original comma/comment tail.
proc t11_set_param {text name value} {
  set out {}
  set n 0
  set pat [format {^(\s*parameter(?:\s+\[[^\]]+\])?\s+(?:integer\s+)?%s\s*=\s*)(\S+?)(\s*[,;].*)$} $name]
  foreach line [split $text "\n"] {
    if {[regexp $pat $line -> pre old rest]} {
      set old_trim [string trimright $old ","]
      set val_trim [string trimright $value ","]
      if {$old_trim ne $val_trim} {
        set line "${pre}${value}${rest}"
        incr n
      }
    }
    lappend out $line
  }
  return [list [join $out "\n"] $n]
}

proc t11_ensure_pcie2_overrides {text} {
  if {[regexp {\.EXT_CFG_CAP_PTR\s*\(\s*6'h1A\s*\)} $text] &&
      [regexp {\.INTERRUPT_PIN\s*\(\s*8'h0*1\s*\)} $text] &&
      [regexp {\.CMD_INTX_IMPLEMENTED\s*\(\s*"TRUE"\s*\)} $text] &&
      [regexp {\.MSI_CAP_MULTIMSGCAP\s*\(\s*3\s*\)} $text]} {
    return [list $text 0]
  }
  set from {pcie_7x_0_core_top  # (
    .LINK_CAP_MAX_LINK_WIDTH (LINK_CAP_MAX_LINK_WIDTH),
    .C_DATA_WIDTH (C_DATA_WIDTH),
    .KEEP_WIDTH (KEEP_WIDTH)
    ) inst}
  set to {pcie_7x_0_core_top  # (
    .LINK_CAP_MAX_LINK_WIDTH (LINK_CAP_MAX_LINK_WIDTH),
    .C_DATA_WIDTH (C_DATA_WIDTH),
    .KEEP_WIDTH (KEEP_WIDTH),
    .PM_CSR_NOSOFTRST (no_soft_rst),
    .CMD_INTX_IMPLEMENTED ("TRUE"),
    .INTERRUPT_PIN (8'h1),
    .EXT_CFG_CAP_PTR (6'h1A),
    .MSI_CAP_MULTIMSGCAP (3),
    .MSI_CAP_PER_VECTOR_MASKING_CAPABLE ("FALSE")
    ) inst}
  if {[string first $from $text] >= 0} {
    set text [string map [list $from $to] $text]
    return [list $text 1]
  }
  # Already has KEEP_WIDTH + PM_CSR but missing identity overrides.
  set from2 {    .KEEP_WIDTH (KEEP_WIDTH),
    .PM_CSR_NOSOFTRST (no_soft_rst)
    ) inst}
  set to2 {    .KEEP_WIDTH (KEEP_WIDTH),
    .PM_CSR_NOSOFTRST (no_soft_rst),
    .CMD_INTX_IMPLEMENTED ("TRUE"),
    .INTERRUPT_PIN (8'h1),
    .EXT_CFG_CAP_PTR (6'h1A),
    .MSI_CAP_MULTIMSGCAP (3),
    .MSI_CAP_PER_VECTOR_MASKING_CAPABLE ("FALSE")
    ) inst}
  if {[string first $from2 $text] >= 0} {
    set text [string map [list $from2 $to2] $text]
    return [list $text 1]
  }
  # Re-patch path: override block exists from an older patch run with the
  # MSI-only values; flip just those two values in place.
  if {[string first {.CMD_INTX_IMPLEMENTED ("FALSE")} $text] >= 0 &&
      [string first {.INTERRUPT_PIN (8'h00)} $text] >= 0} {
    set text [string map [list {.CMD_INTX_IMPLEMENTED ("FALSE")} {.CMD_INTX_IMPLEMENTED ("TRUE")}] $text]
    set text [string map [list {.INTERRUPT_PIN (8'h00)} {.INTERRUPT_PIN (8'h1)}] $text]
    return [list $text 1]
  }
  return [list $text 0]
}

proc t11_ensure_trnd_pad {text} {
  if {[string first {128-C_DATA_WIDTH} $text] >= 0} {
    return [list $text 0]
  }
  set from1 {.TRNTD                               (trn_td                                     )}
  set to1   {.TRNTD                               ({{(128-C_DATA_WIDTH){1'b0}},trn_td}          )}
  set from2 {.TRNTREM                             (trn_trem                                   )}
  set to2   {.TRNTREM                             ({1'b0,trn_trem}          )}
  set n 0
  if {[string first $from1 $text] >= 0} {
    set text [string map [list $from1 $to1] $text]
    incr n
  }
  if {[string first $from2 $text] >= 0} {
    set text [string map [list $from2 $to2] $text]
    incr n
  }
  return [list $text $n]
}

proc t11_patch_core_top {text} {
  set total 0
  foreach {name value} {
    CFG_DEV_ID 16'h1677
    CFG_VEND_ID 16'h14E4
    CFG_REV_ID 8'h11
    CFG_SUBSYS_ID 16'h1677
    CFG_SUBSYS_VEND_ID 16'h14E4
    CLASS_CODE 24'h020000
    BAR0 32'hFFFF0004
    INTERRUPT_PIN 8'h1
    CMD_INTX_IMPLEMENTED {"TRUE"}
    EXT_CFG_CAP_PTR 6'h1A
    EXT_CFG_XP_CAP_PTR 10'h3FF
    PM_CAP_NEXTPTR 8'h58
    MSI_CAP_MULTIMSGCAP 3
    MSI_CAP_PER_VECTOR_MASKING_CAPABLE {"FALSE"}
    DEV_CAP_ENDPOINT_L0S_LATENCY 6
    LINK_CAP_L0S_EXIT_LATENCY_COMCLK_GEN1 6
    LINK_CAP_L0S_EXIT_LATENCY_COMCLK_GEN2 6
    LINK_CAP_L0S_EXIT_LATENCY_GEN1 6
    LINK_CAP_L0S_EXIT_LATENCY_GEN2 6
    LINK_CAP_L1_EXIT_LATENCY_COMCLK_GEN1 6
    LINK_CAP_L1_EXIT_LATENCY_COMCLK_GEN2 6
    LINK_CAP_L1_EXIT_LATENCY_GEN1 6
    LINK_CAP_L1_EXIT_LATENCY_GEN2 6
    LINK_CAP_MAX_LINK_SPEED 4'h1
    LINK_CAP_MAX_LINK_WIDTH 6'h1
    LINK_CTRL2_TARGET_LINK_SPEED 4'h0
    PIPE_PIPELINE_STAGES 0
    DEV_CAP_MAX_PAYLOAD_SUPPORTED 0
    C_DATA_WIDTH 64
    USER_CLK_FREQ 1
    USER_CLK2_DIV2 {"FALSE"}
    TRN_DW {"FALSE"}
    DISABLE_LANE_REVERSAL {"TRUE"}
    DSN_CAP_ON {"FALSE"}
    VC_CAP_ON {"TRUE"}
    AER_CAP_ON {"TRUE"}
    PCIE_GT_DEVICE {"GTP"}
    LTSSM_MAX_LINK_WIDTH 6'h1
    PM_CAP_PMESUPPORT 5'h08
  } {
    lassign [t11_set_param $text $name $value] text n
    incr total $n
  }
  return [list $text $total]
}

proc t11_patch_pcie2 {text} {
  set total 0
  lassign [t11_set_param $text c_pm_cap_next_ptr {"58"}] text n
  incr total $n
  lassign [t11_ensure_pcie2_overrides $text] text n
  incr total $n
  return [list $text $total]
}

proc t11_patch_pcie_top {text} {
  return [t11_set_param $text PM_CAP_NEXTPTR 8'h58]
}

proc t11_patch_pcie_7x {text} {
  set total 0
  lassign [t11_set_param $text PM_CAP_NEXTPTR 8'h58] text n
  incr total $n
  lassign [t11_ensure_trnd_pad $text] text n
  incr total $n
  return [list $text $total]
}

proc t11_patch_file {path} {
  set name [file tail $path]
  set text [t11_read $path]
  switch -exact -- $name {
    pcie_7x_0_core_top.v  { lassign [t11_patch_core_top $text] text n }
    pcie_7x_0_pcie2_top.v { lassign [t11_patch_pcie2 $text] text n }
    pcie_7x_0_pcie_top.v  { lassign [t11_patch_pcie_top $text] text n }
    pcie_7x_0_pcie_7x.v   { lassign [t11_patch_pcie_7x $text] text n }
    default { return 0 }
  }
  if {$n > 0} {
    t11_write $path $text
    puts "T11 patched $n site(s) in $path"
  } else {
    puts "T11 already current: $path"
  }
  return $n
}

proc t11_find_under {dir pattern} {
  set result {}
  if {![file isdirectory $dir]} {
    return $result
  }
  foreach f [glob -nocomplain -directory $dir *] {
    if {[file isdirectory $f]} {
      if {[string equal -nocase [file tail $f] "zdma"]} {
        continue
      }
      set result [concat $result [t11_find_under $f $pattern]]
    } elseif {[string match $pattern [file tail $f]]} {
      lappend result $f
    }
  }
  return $result
}

proc t11_overlay {src dst} {
  if {[file normalize $src] eq [file normalize $dst]} {
    return 0
  }
  file copy -force $src $dst
  puts "T11 overlay [file tail $src] -> $dst"
  return 1
}

proc t11_verify_text {label text} {
  set failed 0
  set checks [list]
  switch -glob -- $label {
    *core_top.v {
      lappend checks {CFG_DEV_ID\s*=\s*16'h1677} "CFG_DEV_ID=1677"
      lappend checks {CFG_VEND_ID\s*=\s*16'h14E4} "CFG_VEND_ID=14E4"
      lappend checks {CFG_REV_ID\s*=\s*8'h11} "CFG_REV_ID=11"
      lappend checks {BAR0\s*=\s*32'hFFFF0004} "BAR0=64KB 64-bit"
      lappend checks {INTERRUPT_PIN\s*=\s*8'h0*1\s*,} "INTERRUPT_PIN=1"
      lappend checks {DEV_CAP_ENDPOINT_L0S_LATENCY\s*=\s*6} "L0s latency=6"
      lappend checks {LINK_CAP_L0S_EXIT_LATENCY_GEN1\s*=\s*6} "L0s exit=6"
      lappend checks {LINK_CAP_L1_EXIT_LATENCY_GEN1\s*=\s*6} "L1 exit=6"
      lappend checks {EXT_CFG_CAP_PTR\s*=\s*6'h1A} "EXT_CFG_CAP_PTR=1A"
      lappend checks {PM_CAP_NEXTPTR\s*=\s*8'h58} "PM_CAP_NEXTPTR=58"
      lappend checks {LINK_CAP_MAX_LINK_SPEED\s*=\s*4'h1} "link speed 2.5GT"
      lappend checks {LINK_CTRL2_TARGET_LINK_SPEED\s*=\s*4'h0} "target speed 4'h0"
      lappend checks {CMD_INTX_IMPLEMENTED\s*=\s*"TRUE"} "CMD_INTX=TRUE"
      lappend checks {MSI_CAP_MULTIMSGCAP\s*=\s*3} "MSI 8 vectors"
    }
    *pcie2_top.v {
      lappend checks {\.INTERRUPT_PIN\s*\(\s*8'h0*1\s*\)} "pcie2 INTERRUPT_PIN=1"
      lappend checks {\.CMD_INTX_IMPLEMENTED\s*\(\s*"TRUE"\s*\)} "pcie2 CMD_INTX overlay"
      lappend checks {\.EXT_CFG_CAP_PTR\s*\(\s*6'h1A\s*\)} "pcie2 EXT_CFG overlay"
      lappend checks {\.MSI_CAP_MULTIMSGCAP\s*\(\s*3\s*\)} "pcie2 MSI 8 overlay"
      lappend checks {c_pm_cap_next_ptr\s*=\s*"58"} "pcie2 PM next=58"
    }
    *pcie_top.v -
    *pcie_7x.v {
      lappend checks {PM_CAP_NEXTPTR\s*=\s*8'h58} "PM_CAP_NEXTPTR=58"
    }
  }
  foreach {pat name} $checks {
    if {![regexp $pat $text]} {
      puts "T11 VERIFY FAIL  $name  ($label)"
      incr failed
    }
  }
  return $failed
}

proc t11_apply_pcie_core_patch {} {
  set repo [t11_repo]
  set srcdir [file join $repo pcie_7x]
  puts "T11 repo=$repo"
  if {![file isdirectory $srcdir]} {
    error "T11: missing $srcdir"
  }

  set patched 0
  foreach f $::t11::files {
    set path [file join $srcdir $f]
    if {![file exists $path]} {
      error "T11: missing $path"
    }
    incr patched [t11_patch_file $path]
  }

  set overlay_roots [list \
    [file join $repo pcileech_enigma_x1] \
    [file join $repo pcileech_100t484_x1] \
  ]
  if {[info exists ::_xil_proj_name_]} {
    lappend overlay_roots [file join $repo $::_xil_proj_name_]
  }
  if {[info exists ::argv]} {
    for {set i 0} {$i < [llength $::argv]} {incr i} {
      if {[lindex $::argv $i] eq "--project_name" && $i+1 < [llength $::argv]} {
        lappend overlay_roots [file join $repo [lindex $::argv [expr {$i+1}]]]
      }
    }
  }

  set seen [dict create]
  foreach root $overlay_roots {
    set root [file normalize $root]
    if {[dict exists $seen $root]} {
      continue
    }
    dict set seen $root 1
    if {![file isdirectory $root]} {
      continue
    }
    foreach f $::t11::files {
      set src [file join $srcdir $f]
      foreach dst [t11_find_under $root $f] {
        if {[t11_is_zdma $dst]} {
          continue
        }
        t11_overlay $src $dst
      }
    }
  }

  set failed 0
  foreach f $::t11::files {
    set path [file join $srcdir $f]
    incr failed [t11_verify_text $f [t11_read $path]]
  }
  if {$failed} {
    error "T11: identity verify failed ($failed check(s))"
  }

  set py [auto_execok python]
  if {$py eq ""} {
    set py [auto_execok py]
  }
  set checker [file join $repo tb check_pcie_core_identity.py]
  if {$py ne "" && [file exists $checker]} {
    if {[catch {exec {*}$py -I $checker} out]} {
      puts "WARNING: T11 python checker skipped/failed: $out"
    } else {
      puts $out
    }
  }

  puts "T11 pcie_core_patch OK (surgical sites=$patched)"
  return 0
}

if {![info exists ::t11_skip_autorun]} {
  if {[catch {t11_apply_pcie_core_patch} err]} {
    puts "ERROR: $err"
    if {[info exists ::origin_dir] && [file isdirectory [file join $::origin_dir pcie_7x]]} {
      error $err
    }
    exit 1
  }
  if {![info exists ::origin_dir] || ![file isdirectory [file join $::origin_dir pcie_7x]]} {
    exit 0
  }
}
