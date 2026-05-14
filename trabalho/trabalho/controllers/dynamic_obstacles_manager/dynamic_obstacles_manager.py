from controller import Supervisor

TIME_STEP = 32
DYNAMIC_ENVIRONMENT = True

# Tempo em que a pessoa aparece e se move.
ACTIVE_TIME = 10.0

# Tempo em que a pessoa desaparece.
HIDDEN_TIME = 20.0

# Duração de um ciclo completo.
TOTAL_CYCLE_TIME = ACTIVE_TIME + HIDDEN_TIME


class DynamicObstaclesManager:
    def __init__(self):
        self.robot = Supervisor()
        self.time_step = int(self.robot.getBasicTimeStep()) or TIME_STEP

        self.obstacle = self.robot.getFromDef("MOVING_PERSON_1")

        if self.obstacle is None:
            print("[dynamic_obstacles] ERROR: MOVING_PERSON_1 not found.")
            self.translation_field = None
        else:
            print("[dynamic_obstacles] MOVING_PERSON_1 found.")
            self.translation_field = self.obstacle.getField("translation")

        # Coordenadas baseadas em corner_plant_1 e corner_plant_2.
        self.point_a = (-0.576, 0.48, 0.05)
        self.point_b = (0.576, 0.48, 0.05)

        # Posição escondida, abaixo do chão.
        self.hidden_pos = (0.0, 0.0, -2.0)

    def interpolate(self, a, b, alpha):
        return [
            a[0] + (b[0] - a[0]) * alpha,
            a[1] + (b[1] - a[1]) * alpha,
            a[2] + (b[2] - a[2]) * alpha,
        ]

    def run(self):
        while self.robot.step(self.time_step) != -1:
            if self.translation_field is None:
                continue

            if not DYNAMIC_ENVIRONMENT:
                self.translation_field.setSFVec3f(list(self.hidden_pos))
                continue

            t = self.robot.getTime()
            cycle_time = t % TOTAL_CYCLE_TIME

            # Durante HIDDEN_TIME, a pessoa desaparece.
            if cycle_time >= ACTIVE_TIME:
                self.translation_field.setSFVec3f(list(self.hidden_pos))
                continue

            # Durante ACTIVE_TIME, a pessoa aparece e move-se.
            phase = cycle_time / ACTIVE_TIME

            if phase < 0.5:
                alpha = phase / 0.5
                pos = self.interpolate(self.point_a, self.point_b, alpha)
            else:
                alpha = (phase - 0.5) / 0.5
                pos = self.interpolate(self.point_b, self.point_a, alpha)

            self.translation_field.setSFVec3f(pos)


if __name__ == "__main__":
    DynamicObstaclesManager().run()