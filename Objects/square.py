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

class Square:
    def __init__(self, x, y):
        self._x = x
        self._y = y
        self._discovered = False
        self._assigned_to_cluster = False

    def draw(self, sf):
        if self._discovered != True:
            rect = pygame.Rect((xm_to_pygame(self._x+0.5), ym_to_pygame(self._y+1.5)), (xm_to_pygame(1), xm_to_pygame(1)))
            pygame.draw.rect(sf, (230, 230, 230), rect)
    
    @property
    def discovered(self):
        return self._discovered

    @discovered.setter
    def discovered(self, value):
        self._discovered = value

    @property
    def location(self):
        return (self._x, self._y)
    
    @property
    def assigned_to_cluster(self):
        return self._assigned_to_cluster
    
    @assigned_to_cluster.setter
    def assigned_to_cluster(self, value):
        self._assigned_to_cluster = value