# === Debugging Print ===
        print(f"[ACC] Mode: {acc_mode.name} | ACC Accel: {acc_accel:.2f} m/s²")
        print(f"[AEB] Mode: {aeb_mode.name} | AEB Accel: {aeb_accel:.2f} m/s²")
        print(f"[LFA] Mode: {lfa_mode.name} | Steer_PID: {steer_pid:.2f}° | Steer_Stanley: {steer_sta:.2f}° | Final Steer: {steer_cmd:.2f}°")
        #print(f"[CTRL] Throttle: {self.control.throttle:.2f} | Brake: {self.control.brake:.2f} | Steer: {self.control.steer:.2f}")
        print(f"[Filtered] {len(filtered)} objects")
        if filtered:
            for i, obj in enumerate(filtered):
                print(
                    f"  F{i}: ID={obj.object_id}, dist={obj.distance:.2f}, status={obj.status.name}, cell={obj.cell_id}")

        print(f"[Predicted] {len(predicted)} objects")
        print(
            f"[ACC Target] ID={acc_target.object_id}, Dist={acc_target.distance:.2f}, Mode={acc_mode.name}, Accel={acc_accel:.2f}")