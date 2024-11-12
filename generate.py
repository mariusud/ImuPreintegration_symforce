import symforce

symforce.set_epsilon_to_symbol()
import symforce.symbolic as sf
from symforce import typing as T
from symforce.values import Values
import sym
import numpy as np
from utils import build_cube_values
from pathlib import Path

from symforce.codegen import (
    CppConfig,
    Codegen,
    values_codegen,
    CodegenConfig,
)
import re
import textwrap

def bias_between_residual( time_delta: sf.V1,
     prev_bias: sf.V3 ,
                next_bias: sf.V3): 
                return ...




def 