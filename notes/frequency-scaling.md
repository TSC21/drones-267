CPU Frequency Scaling
=====================

An easy optimisation that makes the behaviour of an embedded board a little more suitable for real-time processing is to set the 'performance' governor. A comprehensive review is [here][1].

You need to install the `cpufrequtils` package:
```
sudo apt-get install cpufrequtils
```

The current and other available governors are listed through the `cpufreq-info` command. You can set a specific governor by:
```
sudo cpufreq-set -g performance
```

And you can reset it to the ondemand governor by:
```
sudo cpufreq-set -g ondemand
```

[1] https://wiki.debian.org/HowTo/CpuFrequencyScaling