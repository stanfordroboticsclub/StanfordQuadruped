def phase_index(ticks, gaitparams):
    """Calculates which part of the gait cycle the robot should be in given the time in ticks.
    
    Parameters
    ----------
    ticks : int
        Number of timesteps since the program started
    gaitparams : GaitParams
        GaitParams object
    
    Returns
    -------
    Int
        The index of the gait phase that the robot should be in.
    """
    phase_time = ticks % gaitparams.phase_length
    phase_sum = 0
    for i in range(gaitparams.num_phases):
        phase_sum += gaitparams.phase_times[i]
        if phase_time < phase_sum:
            return i
    assert False


def subphase_time(ticks, gaitparams):
    """Calculates the number of ticks (timesteps) since the start of the current phase.

    Parameters
    ----------
    ticks : Int
        Number of timesteps since the program started
    gaitparams : GaitParams
        GaitParams object
    
    Returns
    -------
    Int
        Number of ticks since the start of the current phase.
    """
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
    """Calculates which feet should be in contact at the given number of ticks
    
    Parameters
    ----------
    ticks : Int
        Number of timesteps since the program started.
    gaitparams : GaitParams
        GaitParams object
    
    Returns
    -------
    numpy array (4,)
        Numpy vector with 0 indicating flight and 1 indicating stance.
    """
    return gaitparams.contact_phases[:, phase_index(ticks, gaitparams)]
