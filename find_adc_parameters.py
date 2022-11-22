import numpy as np

def find_factors(num, mx_or_ab, closestn=3):
    """
    A function to find factor pairs that are cloest to a given number.

    Useful for finding clock divisors when faced with multiple options for ADC configurations.
    """
    if np.iterable(mx_or_ab):
        a, b = mx_or_ab
    else:
        a = b = np.arange(1, mx_or_ab)
    
    m = np.outer(a, b)
    if a is b:
        m = m - np.tril(m, -1)
    res = np.abs(m - num)
    closest = np.sort(res.ravel())[:closestn]
    idxs = np.where(np.isin(res, closest))
    return {(e1,e2): e1*e2 for e1, e2 in zip(a[idxs[0]], b[idxs[1]])}

def find_adc_configs(clock, targetfreq, adcdiv=4, minclockdiv=6, nchan=2, minaccum_power=0):
    target = clock/adcdiv/targetfreq/nchan
    gclock_divs = np.arange(minclockdiv, 50)
    clocks_per_conv = np.arange(8, 15, .5)  # assuming gain that yields an added cycle for conversion
    accums = 2**np.arange(minaccum_power, 10)
    factors = [find_factors(target/accum, (gclock_divs, clocks_per_conv), closestn=5) for accum in accums]
    flist = []
    result = []
    for e,a in zip(factors, accums):
        for k,v in e.items():
            flist.append((k[0], k[1], a))
            result.append(v*a)
    result = np.array(result)
    sorti = np.argsort(np.abs(result-target))
    # each element is factors, fraction of closeness to target, resulting sample rate per channel
    return [(flist[i], (result[i]-target)/target, target*targetfreq/result[i]) for i in sorti]

if __name__ == '__main__':

    print("48 MHz / DIV4 -> 24 kHz:")
    for entry in find_adc_configs(48e6, 24e3)[:20]:
        print(entry)

    print("\n48 MHz / DIV4 -> 22.05 kHz:")
    for entry in find_adc_configs(48e6, 22.05e3)[:20]:
        print(entry)
