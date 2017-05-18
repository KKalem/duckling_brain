# -*- coding: utf-8 -*-
"""
Created on Wed May 17 15:33:41 2017

@author: ozer
"""

import pyproj
import numpy as np
from geopy.distance import vincenty


POLY = [[59.349175, 18.071001],
            [59.348092, 18.071087],
            [59.348065, 18.073040],
            [59.349124, 18.072647]]

latmin,lonmin = list(np.min(POLY,axis=0))
latmax,lonmax = list(np.max(POLY,axis=0))

latmid = (latmin+latmax) /2.
lonmid = (lonmin+lonmax) /2.

lonsize = vincenty([latmax,lonmid],[latmin,lonmid]).meters
latsize = vincenty([latmid,lonmax],[latmid,lonmin]).meters

proj = pyproj.Proj(proj='lcc', R= 6371200, lat_1 = latmid, lat_2 = latmid, lon_0 = lonmid)

p = map(lambda p: proj(p[1],p[0]), POLY)



