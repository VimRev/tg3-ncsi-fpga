# Third-Party Attribution and Rights Notices

| Scope | Observed provenance | Treatment in this preview |
| --- | --- | --- |
| PCILeech base HDL | Existing Ulf Frisk / PCILeech FPGA comments; [upstream repository](https://github.com/ufrisk/pcileech-fpga) | Preserve original comments; do not claim wholly original authorship or assign an assumed new license |
| `pcie_7x/*.v` | Some files contain Xilinx / AMD copyrights, disclaimers, and licensing terms | Retain existing build dependencies in the private preview; public redistribution requires review |
| `ip/*.xci`, `ip/100t/*.xci` | Vivado IP configurations | Retain as build inputs; vendor IP remains subject to its applicable terms |
| COE/MEM synthesis initialization inputs | Device initialization models from the local project | Do not publish test fixtures or raw packet captures; confirm the identifiers, provenance, and rights for required initialization data |
| DHCP/NCSI documentation | Project RTL and recorded test results; NCSI principles refer to Microsoft documentation | Distinguish simulated results from conditional Windows expectations and owner-reported hardware feedback |

The local `src/pcileech_com_e.v` and ILA HDL are not used by the selected publication entry points and are excluded. Vivado, third-party drivers, downloaded webpages, and vendor installers are not bundled.

This is a component-attribution inventory, not a substitute for the full third-party licenses or vendor authorization. Applicable licenses and redistribution rights must be confirmed before a public release.
