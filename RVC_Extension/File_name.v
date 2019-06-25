//RISCV_hasHazard.v:  allow compression instruction
//RISCV_hasHazard2.v: only allow standard instruction

//CHIP.v:      for compression instruction
//CHIP_base.v: for standard instruction
//CHIP1.v:     for testing compression instruction
//CHIP2.v:     for testing standard instruction

//cache_comp.v:         for compression instruction + dm
//cache_comp_2way.v:    for compression instruction + 2way
//cache_comp_dm_test.v: for testing compression instruction hit rate
//cache_ro.v:           for testing standard instruction hit rate

//Final_tb.v:  for common use
//Final_tb2.v: for checking miss rate