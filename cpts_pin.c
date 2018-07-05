inline unsigned long cpts_pin_index(unsigned long index)
{
    return index & (CPTS_AVERAGE_LEN - 1);
}

inline u32 cpts_pin_ns_to_cycles(struct cpts_pin* pin, u32 period, u64 ns)
{
    return (u32)div_u64(ns * period, 1000000000);
}

inline u32 cpts_pin_weighted_avg(u32 a, u32 a_weight, u32 b, u32 b_weight)
{
  return (u32)div_u64((u64)a * a_weight + (u64)b * b_weight, a_weight + b_weight);
}

static void cpts_pin_set_array(void* a, void* v, int v_size, int l)
{
    int i;
    u8* ptr = a;
    for(; ptr < (u8*)a + v_size * l; ++ptr)
      memcpy(a, v, v_size);
}

static s32 cpts_pin_extts_calculate_deficit(struct cpts_pin *pin)
{
    unsigned long index = pin->perout_state.index;
    return pin->extts_state.capture > (0ul - pin->extts_state.period / 2) ?
            -(s32)(0ul - pin->extts_state.capture) :
            (s32)(pin->extts_state.capture - pin->extts_state.load[index]);
}

static u32 cpts_pin_extts_calculate_period(struct cpts_pin *pin)
{
    unsigned long indexm1;
    unsigned long index = pin->perout_state.index;
    indexm1 = cpts_pin_index(index - 1);
    return 0ul - pin->extts_state.load[indexm1] + pin->extts_state.deficit[index] -
                pin->extts_state.deficit[indexm1]
}

static u32 cpts_pin_extts_calculate_reload(struct cpts_pin *pin)
{
    unsigned long index = pin->perout_state.index;
    return 0ul - (u32)(2 * pin->extts_state.period -
                (0ul - pin->extts_state.load[index]) + pin->extts_state.deficit[index]);
}

static void cpts_pin_extts_set_reload(struct cpts_pin *pin)
{
    unsigned long indexm1;
    unsigned long index = pin->perout_state.index;
    indexp1 = cpts_pin_index(index + 1);

    pin->extts_state.deficit[index] = cpts_pin_extts_calculate_deficit(pin);

    if(pin->extts_state.last_capture_valid) {
        period = cpts_pin_extts_calculate_period(pin);
        pin->extts_state.period = cpts_pin_weighted_avg(pin->extts_state.period, 3, period, 1);
    }
    else {
        cpts_pin_set_array(pin->extts_state.deficit, &pin->extts_state.deficit[index],
                sizeof(pin->extts_state.deficit[index]), CPTS_AVERAGE_LEN);
    }

    pin->extts_state.load[indexp1] = cpts_pin_extts_calculate_reload(pin);
    
    if(pin->extts_state.deficit[index] >= 0) {
        __omap_dm_timer_write(pin->timer, OMAP_TIMER_LOAD_REG, pin->extts_state.load[indexp1],
                pin->timer->posted);
        pin->timer->context.tldr = pin->extts_state.load[indexp1];
    }

    pin->extts_state.index = indexp1;
}

static u32 cpts_pin_perout_calculate_period(struct cpts_pin *pin)
{
    unsigned long indexm1 = cpts_pin_index(pin->perout_state.index - 1);
    return (u32)div_64((u64)(0lu - pin->perout_state.load[indexm1]) * 1000000000,
                  pin->perout_state.capture - pin->perout_state.last_capture);
}

static s32 cpts_pin_perout_calculate_deficit(struct cpts_pin *pin)
{
    u64 tmp;
    div64_u64_rem(pin->perout_state.capture, 1000000000, &tmp);
    tmp = cpts_pin_ns_to_cycles(pin, pin->perout_state.period, tmp);
    return tmp > pin->perout_state.period / 2 ? -(s32)tmp : (s32)tmp ;
}

static u32 cpts_pin_perout_calculate_reload(struct cpts_pin *pin)
{
    unsigned long index = pin->perout_state.index;
    return 0ul - (u32)(2 * pin->perout_state.period - (0ul - pin->perout_state.load[index]) +
            pin->perout_state.deficit[index]);
}

static void cpts_pin_perout_set_reload(struct cpts_pin* pin)
{
    unsigned long index;
    unsigned long indexp1;
    u32 period;

    index = pin->perout_state.index;
    indexp1 = cpts_pin_index(index + 1);

    period = cpts_pin_perout_calculate_period(pin);
    pin->perout_state.period = cpts_pin_weighted_avg(pin->perout_state.period, 3, period, 1);

    pin->perout_state.deficit[index] = cpts_pin_perout_calculate_deficit(pin);
    if(!deficit_valid) {
        cpts_pin_set_array(pin->perout_state.deficit, &pin->perout_state.deficit[index],
                sizeof(pin->perout_state.deficit[index]), CPTS_AVERAGE_LEN);
        deficit_valid = true;
    }
    pin->perout_state.load[indexp1] = cpts_pin_perout_calculate_reload(pin);

    __omap_dm_timer_write(pin->timer, OMAP_TIMER_LOAD_REG, pin->extts_state.load[indexp1],
            pin->timer->posted);
    pin->timer->context.tldr = pin->extts_state.load[indexp1];
    index = indexp1;
}

static void cpts_pin_capture_bottom_half(struct work_struct *work)
{
    unsigned long indexp1;
    struct cpts_pin *pin = container_of(work, struct cpts_pin, capture_work);

    pr_info("cpts: capture hit");

    switch(pin->state.type)
    {
    case PTP_CLK_REQ_EXTTS:

        if (pin->extts_state.last_capture_valid && pin->extts_state.period == 0)
        { // ready to make an initial period calculation
            pin->extts_state.period = pin->extts_state.capture - pin->extts_state.last_capture;
            pin->extts_state.load[0] = 0ul - pin->extts_state.period;

            // write load and load it.
            __omap_dm_timer_write(pin->timer, OMAP_TIMER_LOAD_REG, pin->extts_state.load[0], pin->timer->posted);
            omap_dm_timer_trigger(pin->timer);
            pin->timer->context.tldr = pin->extts_state.load[0];

            pr_info("cpts: capture found pulse, %u %u", pin->extts_state.period, pin->extts_state.load[0]);

            cpts_pin_set_array(pin->extts_state.load, &pin->extts_state.load[0],
                sizeof(pin->extts_state.load[0]), CPTS_AVERAGE_LEN);
            cpts_pin_set_array(pin->extts_state.deficit, &pin->extts_state.load[0],
                sizeof(pin->extts_state.load[0]), CPTS_AVERAGE_LEN);

            pin->extts_state.last_capture_valid = false;
            break;
        }
        else if(pin->extts_state.period != 0)
        { // initial period is calculated, need to fix the load value to match the period
            indexp1 = cpts_pin_index(pin->perout_state.index + 1);
            cpts_pin_extts_set_reload(pin);
            pr_info("cpts: capture %s load value %u", pin->ptp_pin->name, pin->extts_state.load[indexp1]);
        }
        pin->extts_state.last_capture = pin->extts_state.capture;
        pin->extts_state.last_capture_valid = true;
        break;
    case PTP_CLK_REQ_PEROUT:
    default:
        break;
    }
}

static void cpts_pin_overflow_bottom_half(struct work_struct *work)
{
    u32 ctrl;
    s32 avg;
    unsigned long index, indexm1, indexp1;
    struct cpts_pin *pin = container_of(work, struct cpts_pin, overflow_work);
    struct cpts *cpts = container_of(pin, struct cpts, pins[pin->ptp_pin->index]);

    // read out all HW events (could be multiple)
    while(cpts_fifo_read(cpts, CPTS_EV_HW) == 0);

    pr_info("cpts: overflow hit");

    switch(pin->state.type)
    {
    case PTP_CLK_REQ_EXTTS:
        if(pin->extts_state.period != 0)
        {
            __omap_dm_timer_write(pin->timer, OMAP_TIMER_LOAD_REG, pin->extts_state.load[pin->extts_state.index], pin->timer->posted);
            pin->timer->context.tldr = pin->extts_state.load[pin->extts_state.index];
            
            ctrl = __omap_dm_timer_read(pin->timer, OMAP_TIMER_CTRL_REG, pin->timer->posted);

            avg = cpts_extts_deficit_avg(pin);
            pr_info("cpts: overflow deficit avg %d", avg);
            if(avg < 10 && !(ctrl & OMAP_TIMER_CTRL_CE))
            { // clock is close enough, turn on output.
                pr_info("cpts: started timer output, avg %d", avg);
                ctrl |= OMAP_TIMER_CTRL_CE;
                __omap_dm_timer_write(pin->timer, OMAP_TIMER_CTRL_REG, ctrl, pin->timer->posted);
                pin->timer->context.tclr = ctrl;
            }
        }
        break;
    case PTP_CLK_REQ_PEROUT:
        if(pin->perout_state.new_capture) {
            if(pin->perout_state.last_capture_valid)
                cpts_pin_perout_set_reload(pin);

            pin->perout_state.last_capture = pin->perout_state.capture;
            pin->perout_state.last_capture_valid = true;
        }
    default:
        break;
    }
}
