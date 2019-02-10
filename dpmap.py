# file contains collected data

# rect (x,y,w,h)
# X,Y --> BLOCKS ON GRID
# W,H --> METERS (ROUNDED)

DPMAP = {"GYM": (27, 1, 29, 17),
         "ARTS": (5, 5, 16, 10),
         "S1": (11, 21, 10, 4),
         "S2": (5, 22, 4, 8),
         "ADMIN": (5, 32, 11, 4),
         "M": (18, 29, 4, 7),
         "B": (28, 22, 4, 15),
         "T1": (35, 22, 6, 4),
         "T2": (38, 28, 4, 15),
         "T3": (28, 39, 7, 4),
         "LIBRARY": (17, 39, 9, 8),
         "H1": (7, 43, 7, 4),
         "H2": (1, 43, 3, 10),
         "H3": (1, 55, 7, 5),
         "H4": (11, 49, 3, 10),
         "A": (21, 50, 4, 8),
         "CAFETERIA": (34, 50, 4, 14),
         "P": (22, 61, 10, 3),
         "ENGINEERING": (23, 68, 13, 4),
         "Q1": (8, 62, 6, 4),
         "Q2": (1, 63, 5, 1),
         "Q3": (1, 66, 5, 1),
         "Q4": (1, 69, 5, 1),
         "Q5": (1, 72, 5, 1),
         "Q6": (9, 72, 3, 1),
         "BATHROOM": (9, 67, 0, 5),
         "TRACK1": (48, 21, 9, 28),
         "TRACK2": (45, 46, 3, 3),
         "TRACK3": (44, 49, 13, 25)
         }
CLASSES = {"ENGINEERING": (26, 67),
           "PHYSICS": (17, 30),
           "ENGLISH": (8, 72),
           "LATIN": (10, 54),
           "MATH": (13, 42)
           }

MAPSIZE = (58, 75)
METERSTOCOORDINATES = 1/4   # 1 BLOCK ON THE GRID (PICTURE) = 4 METERS (REAL)
