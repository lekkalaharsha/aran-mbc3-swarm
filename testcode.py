import asyncio
import numpy as np
import cvxpy as cp

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw
from mavsdk.action import ActionError


# =========================
# PX4 STATUS DECODER (LIVE)
# =========================
async def px4_status_monitor(drone):
    async for msg in drone.telemetry.status_text():
        print(f"[PX4] {msg.type.name}: {msg.text}")


# =========================
# HEALTH WAIT
# =========================
async def wait_for_ready(drone):
    print("Waiting for full system readiness...")

    async for health in drone.telemetry.health():
        print(f"GPS:{health.is_global_position_ok} "
              f"HOME:{health.is_home_position_ok} "
              f"LOCAL:{health.is_local_position_ok}")

        if (health.is_global_position_ok and
            health.is_home_position_ok and
            health.is_local_position_ok):
            print("System ready for arming")
            return

        await asyncio.sleep(0.5)


# =========================
# ARM WITH RETRY
# =========================
async def arm_with_retry(drone, retries=5):
    for i in range(retries):
        try:
            print(f"Arming attempt {i+1}...")
            await drone.action.arm()

            # confirm armed
            async for armed in drone.telemetry.armed():
                if armed:
                    print("ARMED confirmed")
                    return True
                break

        except ActionError as e:
            print(f"Arm failed: {e}")
            print("Retrying...\n")
            await asyncio.sleep(2)

    print("❌ Failed to arm after retries")
    return False


# =========================
# MPC
# =========================
class MPC3D:
    def __init__(self, dt=0.1, horizon=10):
        self.dt = dt
        self.N = horizon
        self.Qp = 25.0
        self.Qv = 8.0
        self.R  = 0.5

    def solve(self, state, ref):
        x = cp.Variable((6, self.N+1))
        u = cp.Variable((3, self.N))

        cost = 0
        cons = [x[:,0] == state]

        for k in range(self.N):
            cons += [
                x[0,k+1] == x[0,k] + u[0,k]*self.dt,
                x[1,k+1] == x[1,k] + u[1,k]*self.dt,
                x[2,k+1] == x[2,k] + u[2,k]*self.dt,
                x[3,k+1] == u[0,k],
                x[4,k+1] == u[1,k],
                x[5,k+1] == u[2,k],
            ]

            cost += self.Qp * cp.sum_squares(x[0:3,k] - ref[0:3])
            cost += self.Qv * cp.sum_squares(x[3:6,k])
            cost += self.R  * cp.sum_squares(u[:,k])

            cons += [cp.norm(u[:,k], "inf") <= 4.0]

        prob = cp.Problem(cp.Minimize(cost), cons)
        prob.solve(solver=cp.OSQP, warm_start=True)

        if u.value is None:
            return np.zeros(3)

        return u.value[:,0]


# =========================
# TRAJECTORY
# =========================
def smoothstep(t):
    return 3*t**2 - 2*t**3

def trajectory(t):
    if t < 3:
        z = -20 * smoothstep(t/3)
    elif t < 7:
        z = -20
    elif t < 11:
        z = -20 * (1 - smoothstep((t-7)/4))
    else:
        z = 0
    return np.array([0,0,z,0,0,0])


# =========================
# MAIN
# =========================
async def run():
    drone = System()

    # ✅ FIX: proper connection
    await drone.connect(system_address="udpin://:14540")

    print("Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected")
            break

    # 🔥 Start PX4 message monitor
    asyncio.create_task(px4_status_monitor(drone))

    # ✅ Wait full readiness
    await wait_for_ready(drone)

    # ✅ Arm with retry
    success = await arm_with_retry(drone)
    if not success:
        return

    # =========================
    # TAKEOFF
    # =========================
    print("Taking off...")
    await drone.action.set_takeoff_altitude(5.0)
    await drone.action.takeoff()

    async for pos in drone.telemetry.position():
        if pos.relative_altitude_m > 2.0:
            print("Airborne confirmed")
            break
        await asyncio.sleep(0.1)

    # =========================
    # OFFBOARD INIT
    # =========================
    for _ in range(20):
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(0,0,0,0)
        )
        await asyncio.sleep(0.05)

    print("Starting Offboard...")
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Offboard failed: {e}")
        return

    # =========================
    # MPC LOOP
    # =========================
    mpc = MPC3D()
    t = 0.0
    dt = 0.1

    print("Running MPC...")

    async for pv in drone.telemetry.position_velocity_ned():
        x = pv.position.north_m
        y = pv.position.east_m
        z = pv.position.down_m

        vx = pv.velocity.north_m_s
        vy = pv.velocity.east_m_s
        vz = pv.velocity.down_m_s

        state = np.array([x,y,z,vx,vy,vz])
        ref = trajectory(t)

        vel_cmd = mpc.solve(state, ref)

        # lock XY
        vel_cmd[0] = 0.0
        vel_cmd[1] = 0.0
        vel_cmd[2] = float(np.clip(vel_cmd[2], -3, 3))

        print(f"t={t:.1f} z={z:.2f} vz_cmd={vel_cmd[2]:.2f}")

        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(0,0,vel_cmd[2],0)
        )

        t += dt
        if t > 12:
            break

        await asyncio.sleep(dt)

    print("Stopping Offboard...")
    await drone.offboard.stop()

    print("Landing...")
    await drone.action.land()


if __name__ == "__main__":
    asyncio.run(run())