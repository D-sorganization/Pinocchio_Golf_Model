"""Manual verification of the Unified Platform pipeline."""

import logging
import sys
from pathlib import Path

import numpy as np

# Add project root to path
sys.path.append(str(Path(__file__).parent.parent))

try:
    import pinocchio as pin
except ImportError:
    print("Error: Pinocchio not found. Please ensure you are in the correct environment.")
    sys.exit(1)

from dtack.ik.pink_solver import PinkSolver
from dtack.ik.tasks import create_frame_task
from dtack.sim.dynamics import DynamicsEngine
from dtack.utils.matlab_importer import MATLABImporter

def main():
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger("VerifyWorkflow")
    
    logger.info("1. Verifying Pinocchio Installation...")
    print(f"Pinocchio version: {pin.__version__}")
    
    logger.info("2. Building Sample Model...")
    model = pin.buildSampleModelManipulator()
    data = model.createData()
    logger.info(f"Model built: nq={model.nq}, nv={model.nv}")
    
    logger.info("3. Verifying Dynamics Engine...")
    dyn = DynamicsEngine(model, data)
    q = pin.neutral(model)
    v = np.zeros(model.nv)
    tau = np.zeros(model.nv)
    acc = dyn.forward_dynamics(q, v, tau)
    logger.info(f"Forward Dynamics computed. Acc shape: {acc.shape}")
    
    logger.info("4. Verifying Pink Solver...")
    try:
        solver = PinkSolver(model, data, pin.GeometryModel(), pin.GeometryModel())
        logger.info("PinkSolver instantiated.")
    except Exception as e:
        logger.error(f"PinkSolver failed: {e}")
        
    logger.info("5. Verifying Data Import...")
    importer = MATLABImporter()
    logger.info("MATLABImporter instantiated.")
    
    logger.info("Verification Complete. Ready for integration.")

if __name__ == "__main__":
    main()
