#!/usr/bin/env python
# -*- coding: utf-8 -*-
import subprocess
import commands
import decimal

freqency = 200*1000*1000

cmd = """awk '{for(i=1;i<NF;i++){printf $i" "}}' teraterm.log > test.txt"""
subprocess.call(cmd, shell=True)

time = commands.getoutput("awk '{print $4097}' teraterm.log")
print "elasped time: %s sec" % str(decimal.Decimal(int(time, 16))/decimal.Decimal(freqency))
