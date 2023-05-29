# Copyright (c) 2023-2024 Pelle Wiersma.
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:

# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import pygame
import numpy as np
from Util.util import m_to_pygame, xm_to_pygame, ym_to_pygame

class Cluster:
    def __init__(self, squares):
        self._assigned = False
        self._squares = squares
        self._location = self.calc_mean()
        self._mean = (0, 0)

    def draw(self, sf):
        pass

    def calc_mean(self):
        tot_x = 0
        tot_y = 0
        for square in self._squares:
            tot_x += square.location[0]
            tot_y += square.location[1]
        mean = [round(tot_x/len(self._squares))+1, round(tot_y/len(self._squares))+1]
        self._mean = mean
        return mean
    
    @property
    def assigned(self):
        return self._assigned

    @assigned.setter
    def assigned(self, value):
        self._assigned = value

    @property
    def location(self):
        return self._location

    @property
    def mean(self):
        self.calc_mean()
        return self._mean 
    
    def size(self):
        return len(self._squares)