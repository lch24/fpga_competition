// Shared workspace capacity/layout, derived from the public configuration.
    // Four lens-shape residuals follow the observed corner residuals. HOST
    // streams still contain exactly PAR_RESIDUALS values; ROM appends these.
    localparam LM_ROWS=`PAR_RESIDUALS+4,
      DESC=288,CURRENT=DESC+4*`PAR_ACTIVE_N,
      TRIAL=CURRENT+`PAR_STATE_N,BASELINE=TRIAL+`PAR_STATE_N,
      PLUS=BASELINE+LM_ROWS,MINUS=PLUS+LM_ROWS,
      COLUMN=MINUS+LM_ROWS,JAC=COLUMN+LM_ROWS,
      NORMAL=JAC+`PAR_JAC_BANKS*LM_ROWS,AUGMENTED=NORMAL+`PAR_ACTIVE_N*`PAR_ACTIVE_N,
      SCALES=AUGMENTED+`PAR_GAUSS_SIZE,DELTA=SCALES+`PAR_ACTIVE_N,
      GRADIENT=DELTA+`PAR_ACTIVE_N,WORDS=1<<$clog2(GRADIENT+`PAR_ACTIVE_N);
