import sklearn
import os
import numpy as np
from sklearn.ensemble import RandomForestClassifier
import joblib

def GetPartition(C0,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14,C15,C16,C17,C18,C19,C20,C21
                 ,C22,C23,C24,C25,C26):
  Character = np.array([C0,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14,C15,C16,C17,C18,C19,C20,C21
                 ,C22,C23,C24,C25])
  Character = Character.reshape(-1, 26)

  # 0: simple  1: fuzzy  2: complex
  if C26 == 1:
    lr = joblib.load("Termination_32.pkl")
    P = lr.predict(Character)
    if P == 0:  # Termination
      return 0
    else:       # Not Termination
      return -1
  else:
    lr = joblib.load("Partition_32.pkl")
    P = lr.predict(Character)
    return P

