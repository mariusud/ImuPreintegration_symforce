import symforce

symforce.set_epsilon_to_symbol()
import symforce.symbolic as sf
from symforce import typing as T
from symforce.values import Values
import sym
import numpy as np
from pathlib import Path

from symforce.codegen import (
    CppConfig,
    Codegen,
    values_codegen,
    CodegenConfig,
)
import re
import textwrap


# TODO: scalar of V1 for time delta?
def bias_between_factor_residual(
    diag_sqrt_info: sf.V3, time_delta: sf.Scalar, prev_bias: sf.V3, next_bias: sf.V3
) -> sf.V3:
    """Residual for accelerometer/gyroscope Bias between factor.
    A wiener process (random walk model) where the bias changes at a constant rate
    over time. time_delta is the time between the two bias states.

    residual = (time_delta * diagonal_information) * (bias_i - bias_j)

    """
    sqrt_info = sf.sqrt(time_delta) * sf.M33.diag(diag_sqrt_info)
    return sqrt_info * (prev_bias - next_bias)


def generate_bias_between_factor_residual_code(output_dir: Path) -> None:
    codegen = Codegen.function(bias_between_factor_residual, config=CppConfig())
    codegen_with_linearization = codegen.with_linearization(
        which_args=["prev_bias", "next_bias"]
    )
    codegen_with_linearization.generate_function(
        output_dir=output_dir, skip_directory_nesting=True
    )


def generate_keys(output_dir: Path) -> None:
    values = Values(
        # Pose quantities
        pose=sf.Pose3(),
        measured_pose=sf.Pose3(),
        # IMU quantities
        time_delta=sf.Scalar(),
        velocity=sf.V3(),
        velocity_prior_sqrt_info=sf.M33(),
        accel_bias=sf.V3(),
        gyro_bias=sf.V3(),
        gravity=sf.V3(),
        accel_bias_prior_sqrt_info=sf.M33(),
        gyro_bias_prior_sqrt_info=sf.M33(),
        accel_bias_diag_sqrt_info=sf.V3(),
        gyro_bias_diag_sqrt_info=sf.V3(),
        # Relative pose quantities
        sqrt_info=sf.M66(),
        # Generic
        epsilon=sf.Scalar(),
        # DEPRECATED
    )

    values_codegen.generate_values_keys(
        values, output_dir, config=CppConfig(), skip_directory_nesting=True
    )


def generate(output_dir: Path):
    generate_bias_between_factor_residual_code(output_dir)
    generate_keys(output_dir)


if __name__ == "__main__":
    generate(Path(__file__).parent / "gen")
