def phase_index(ticks, gaitparams):
    phase_time = ticks % gaitparams.phase_length
    phase_sum = 0
    for i in range(gaitparams.num_phases):
        phase_sum += gaitparams.phase_times[i]
        if phase_time < phase_sum:
            return i
    assert False


def subphase_time(ticks, gaitparams):
    phase_time = ticks % gaitparams.phase_length
    phase_sum = 0
    subphase_t = 0
    for i in range(gaitparams.num_phases):
        phase_sum += gaitparams.phase_times[i]
        if phase_time < phase_sum:
            subphase_t = phase_time - phase_sum + gaitparams.phase_times[i]
            return subphase_t
    assert False


def contacts(ticks, gaitparams):
    return gaitparams.contact_phases[:, phase_index(ticks, gaitparams)]
