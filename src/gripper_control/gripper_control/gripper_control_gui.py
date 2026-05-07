import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from dobot_msgs_v4.srv import DO


class GripperControlNode(Node):
    def __init__(self) -> None:
        super().__init__('gripper_control_gui')
        self._do_service_name = self.declare_parameter(
            'do_service',
            '/dobot_bringup_ros2/srv/DO',
        ).value
        self._do_client = self.create_client(DO, self._do_service_name)

    @property
    def do_service_name(self) -> str:
        return self._do_service_name

    def is_service_ready(self) -> bool:
        return self._do_client.service_is_ready()

    def send_do(self, index: int, status: int, on_complete, time_ms: int = 0) -> None:
        request = DO.Request()
        request.index = int(index)
        request.status = int(status)
        request.time = int(time_ms)

        if not self._do_client.wait_for_service(timeout_sec=0.2):
            on_complete(False, -1, f'service unavailable: {self._do_service_name}')
            return

        future = self._do_client.call_async(request)

        def _done(fut):
            try:
                response = fut.result()
            except Exception as ex:  # noqa: BLE001
                on_complete(False, -1, f'call failed: {ex}')
                return

            if response is None:
                on_complete(False, -1, 'empty response')
                return

            result_code = int(getattr(response, 'res', -1))
            success = result_code != -1
            on_complete(success, result_code, f'res={result_code}')

        future.add_done_callback(_done)


class GripperControlApp:
    def __init__(
        self,
        root: tk.Tk,
        node: GripperControlNode,
        executor: SingleThreadedExecutor,
        spin_thread: threading.Thread,
        stop_event: threading.Event,
    ) -> None:
        self._root = root
        self._node = node
        self._executor = executor
        self._spin_thread = spin_thread
        self._stop_event = stop_event

        self._closing = False
        self._finalized = False
        self._channels = {}
        self._grip_button = None
        self._release_button = None

        auto_off_on_exit = self._node.declare_parameter('auto_off_on_exit', True).value
        self._auto_off_on_exit = bool(auto_off_on_exit)

        self._status_var = tk.StringVar(value='Ready')
        self._service_var = tk.StringVar(value='Checking service...')

        self._build_ui()
        self._root.protocol('WM_DELETE_WINDOW', self._on_close)
        self._update_service_indicator()

    def _build_ui(self) -> None:
        self._root.title('Dobot Gripper Control (DO1/DO2/DO3)')
        self._root.geometry('560x330')
        self._root.resizable(False, False)

        outer = ttk.Frame(self._root, padding=14)
        outer.pack(fill='both', expand=True)

        header = ttk.Label(
            outer,
            text='Dobot DO Gripper Control',
            font=('TkDefaultFont', 12, 'bold'),
        )
        header.grid(row=0, column=0, sticky='w')

        svc = ttk.Label(outer, textvariable=self._service_var)
        svc.grid(row=1, column=0, sticky='w', pady=(4, 10))

        panel = ttk.LabelFrame(outer, text='Outputs', padding=10)
        panel.grid(row=2, column=0, sticky='ew')
        panel.columnconfigure(2, weight=1)

        self._create_channel_row(panel, do_index=1, row=0)
        self._create_channel_row(panel, do_index=2, row=1)
        self._create_channel_row(panel, do_index=3, row=2)

        action_panel = ttk.LabelFrame(outer, text='Quick Actions', padding=10)
        action_panel.grid(row=3, column=0, sticky='ew', pady=(10, 0))
        self._grip_button = tk.Button(
            action_panel,
            text='Grip',
            width=12,
            bg='#1f7a1f',
            fg='white',
            activebackground='#2e8b57',
            command=self._on_grip,
        )
        self._grip_button.grid(row=0, column=0, padx=(0, 12))
        self._release_button = tk.Button(
            action_panel,
            text='Release',
            width=12,
            bg='#8b1a1a',
            fg='white',
            activebackground='#b22222',
            command=self._on_release,
        )
        self._release_button.grid(row=0, column=1, padx=(0, 12))

        status = ttk.Label(outer, textvariable=self._status_var, foreground='#204a87')
        status.grid(row=4, column=0, sticky='w', pady=(10, 0))

        hint = ttk.Label(
            outer,
            text='Behavior: 0 ms = keep ON until manual OFF. >0 ms = auto OFF after delay.',
        )
        hint.grid(row=5, column=0, sticky='w', pady=(6, 0))
        self._update_action_buttons()

    def _create_channel_row(self, parent: ttk.LabelFrame, do_index: int, row: int) -> None:
        frame = ttk.Frame(parent)
        frame.grid(row=row, column=0, sticky='ew', pady=4)

        button = tk.Button(
            frame,
            text=f'DO{do_index} OFF',
            width=13,
            bg='#8b1a1a',
            fg='white',
            activebackground='#b22222',
            command=lambda idx=do_index: self._on_toggle(idx),
        )
        button.grid(row=0, column=0, padx=(0, 12))

        ttk.Label(frame, text='Auto OFF (ms)').grid(row=0, column=1, padx=(0, 6))

        duration_var = tk.StringVar(value='0')
        duration_entry = ttk.Entry(frame, textvariable=duration_var, width=10)
        duration_entry.grid(row=0, column=2, padx=(0, 10))

        state_label = tk.Label(frame, text='OFF', width=12, fg='#8b1a1a')
        state_label.grid(row=0, column=3, sticky='w')

        self._channels[do_index] = {
            'button': button,
            'duration_var': duration_var,
            'duration_entry': duration_entry,
            'state_label': state_label,
            'on': False,
            'pending': False,
            'timer_after_id': None,
        }

    def _update_service_indicator(self) -> None:
        if self._closing:
            return

        if self._node.is_service_ready():
            self._service_var.set(f'Service ready: {self._node.do_service_name}')
        else:
            self._service_var.set(f'Waiting for service: {self._node.do_service_name}')

        self._root.after(400, self._update_service_indicator)

    def _set_status(self, text: str) -> None:
        self._status_var.set(text)

    def _set_channel_ui(self, do_index: int) -> None:
        channel = self._channels[do_index]
        button = channel['button']
        state_label = channel['state_label']

        if channel['pending']:
            button.configure(state=tk.DISABLED)
        else:
            button.configure(state=tk.NORMAL)

        if channel['on']:
            button.configure(text=f'DO{do_index} ON', bg='#1f7a1f', activebackground='#2e8b57')
            state_label.configure(text='ON', fg='#1f7a1f')
        else:
            button.configure(text=f'DO{do_index} OFF', bg='#8b1a1a', activebackground='#b22222')
            state_label.configure(text='OFF', fg='#8b1a1a')

        if channel['pending']:
            state_label.configure(text='PENDING', fg='#8a6d1d')
        self._update_action_buttons()

    def _any_channel_pending(self) -> bool:
        return any(channel['pending'] for channel in self._channels.values())

    def _update_action_buttons(self) -> None:
        state = tk.NORMAL
        if self._closing or self._any_channel_pending():
            state = tk.DISABLED
        if self._grip_button is not None:
            self._grip_button.configure(state=state)
        if self._release_button is not None:
            self._release_button.configure(state=state)

    def _parse_duration_ms(self, do_index: int):
        text = self._channels[do_index]['duration_var'].get().strip()
        if text == '':
            text = '0'

        try:
            value = int(text)
        except ValueError:
            self._set_status(f'DO{do_index}: invalid ms value "{text}"')
            return None

        if value < 0:
            self._set_status(f'DO{do_index}: ms must be >= 0')
            return None

        return value

    def _cancel_auto_off_timer(self, do_index: int) -> None:
        channel = self._channels[do_index]
        timer_after_id = channel['timer_after_id']
        if timer_after_id is not None:
            self._root.after_cancel(timer_after_id)
            channel['timer_after_id'] = None

    def _on_toggle(self, do_index: int) -> None:
        channel = self._channels[do_index]
        if channel['pending']:
            return

        if channel['on']:
            self._request_off(do_index, reason='manual')
            return

        duration_ms = self._parse_duration_ms(do_index)
        if duration_ms is None:
            return
        self._request_on(do_index, duration_ms)

    def _on_grip(self) -> None:
        if self._closing or self._any_channel_pending():
            return
        duration_ms_do1 = self._parse_duration_ms(1)
        if duration_ms_do1 is None:
            return
        self._set_status('Grip: DO1 ON + DO3 suction ON')
        self._request_on(1, duration_ms_do1)
        self._request_on(3, 0)

    def _on_release(self) -> None:
        if self._closing or self._any_channel_pending():
            return

        channel_1 = self._channels[1]
        channel_2 = self._channels[2]
        channel_3 = self._channels[3]
        self._cancel_auto_off_timer(1)
        self._cancel_auto_off_timer(2)
        self._cancel_auto_off_timer(3)
        channel_1['pending'] = True
        channel_2['pending'] = True
        channel_3['pending'] = True
        self._set_status('Release: DO1 OFF, DO3 OFF (vent), DO2 pulse 100 ms')
        self._set_channel_ui(1)
        self._set_channel_ui(2)
        self._set_channel_ui(3)

        def _finish_release(status_text: str) -> None:
            channel_1['pending'] = False
            channel_2['pending'] = False
            channel_3['pending'] = False
            self._set_status(status_text)
            self._set_channel_ui(1)
            self._set_channel_ui(2)
            self._set_channel_ui(3)

        def _do2_off() -> None:
            self._send_do(2, 0, _on_do2_off_complete)

        def _on_do2_off_complete(success: bool, result_code: int, detail: str) -> None:
            _ = result_code
            if success:
                channel_2['on'] = False
                _finish_release(f'Release done: DO2 OFF ({detail})')
            else:
                _finish_release(f'Release warning: DO2 OFF failed ({detail})')

        def _on_do2_on_complete(success: bool, result_code: int, detail: str) -> None:
            _ = result_code
            if success:
                channel_2['on'] = True
                self._set_status(f'Release: DO2 ON ({detail}), waiting 100 ms')
                self._set_channel_ui(2)
                self._root.after(100, _do2_off)
            else:
                _finish_release(f'Release failed: DO2 ON failed ({detail})')

        def _on_do3_off_complete(success: bool, result_code: int, detail: str) -> None:
            _ = result_code
            if success:
                channel_3['on'] = False
                self._set_status(f'Release: DO3 OFF / vent ({detail}), pulsing DO2')
            else:
                self._set_status(f'Release warning: DO3 OFF / vent failed ({detail}), pulsing DO2')
            self._set_channel_ui(3)
            self._send_do(2, 1, _on_do2_on_complete)

        def _on_do1_off_complete(success: bool, result_code: int, detail: str) -> None:
            _ = result_code
            if success:
                channel_1['on'] = False
                self._set_status(f'Release: DO1 OFF ({detail}), switching DO3 to vent')
            else:
                self._set_status(f'Release warning: DO1 OFF failed ({detail}), switching DO3 to vent')
            self._set_channel_ui(1)
            self._send_do(3, 0, _on_do3_off_complete)

        self._send_do(1, 0, _on_do1_off_complete)

    def _send_do(self, do_index: int, status: int, on_complete) -> None:
        def _wrapped_complete(success: bool, result_code: int, detail: str) -> None:
            self._root.after(0, lambda: on_complete(success, result_code, detail))

        # Use immediate DO set/reset. Auto-off timing is managed by GUI timer.
        self._node.send_do(do_index, status, _wrapped_complete, time_ms=0)

    def _request_on(self, do_index: int, duration_ms: int) -> None:
        channel = self._channels[do_index]
        channel['pending'] = True
        self._set_channel_ui(do_index)
        self._set_status(f'DO{do_index}: sending ON (1)')

        def _complete(success: bool, result_code: int, detail: str) -> None:
            channel['pending'] = False
            if success:
                channel['on'] = True
                self._set_status(f'DO{do_index}: ON, {detail}')
                if duration_ms > 0:
                    channel['timer_after_id'] = self._root.after(
                        duration_ms,
                        lambda idx=do_index: self._auto_off(idx),
                    )
            else:
                channel['on'] = False
                self._set_status(f'DO{do_index}: ON failed ({detail})')
            self._set_channel_ui(do_index)

        self._send_do(do_index, 1, _complete)

    def _request_off(self, do_index: int, reason: str) -> None:
        channel = self._channels[do_index]
        self._cancel_auto_off_timer(do_index)
        channel['pending'] = True
        self._set_channel_ui(do_index)
        self._set_status(f'DO{do_index}: sending OFF (0), reason={reason}')

        def _complete(success: bool, result_code: int, detail: str) -> None:
            channel['pending'] = False
            if success:
                channel['on'] = False
                self._set_status(f'DO{do_index}: OFF, {detail}')
            else:
                self._set_status(f'DO{do_index}: OFF failed ({detail})')
            self._set_channel_ui(do_index)

        self._send_do(do_index, 0, _complete)

    def _auto_off(self, do_index: int) -> None:
        channel = self._channels[do_index]
        channel['timer_after_id'] = None
        if self._closing or channel['pending'] or not channel['on']:
            return
        self._request_off(do_index, reason='auto')

    def _finalize_shutdown(self) -> None:
        if self._finalized:
            return
        self._finalized = True

        self._stop_event.set()

        try:
            self._executor.shutdown()
        except Exception:  # noqa: BLE001
            pass

        try:
            self._node.destroy_node()
        except Exception:  # noqa: BLE001
            pass

        if rclpy.ok():
            rclpy.shutdown()

        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=1.0)

        self._root.destroy()

    def _on_close(self) -> None:
        if self._closing:
            return
        self._closing = True

        for do_index in (1, 2, 3):
            self._cancel_auto_off_timer(do_index)

        if not self._auto_off_on_exit:
            self._finalize_shutdown()
            return

        active_outputs = [idx for idx, channel in self._channels.items() if channel['on']]
        if not active_outputs:
            self._finalize_shutdown()
            return

        self._set_status('Closing: turning active outputs OFF...')

        pending_count = {'count': len(active_outputs)}

        def _exit_ack() -> None:
            pending_count['count'] -= 1
            if pending_count['count'] <= 0:
                self._finalize_shutdown()

        for do_index in active_outputs:
            def _complete(success: bool, result_code: int, detail: str, idx: int = do_index) -> None:
                _ = success
                _ = result_code
                _ = detail
                self._channels[idx]['on'] = False
                self._channels[idx]['pending'] = False
                self._set_channel_ui(idx)
                _exit_ack()

            self._send_do(do_index, 0, _complete)

        # Hard timeout so shutdown is never blocked by service issues.
        self._root.after(1200, self._finalize_shutdown)


def _spin_executor(executor: SingleThreadedExecutor, stop_event: threading.Event) -> None:
    while rclpy.ok() and not stop_event.is_set():
        executor.spin_once(timeout_sec=0.1)


def main() -> None:
    rclpy.init()

    node = GripperControlNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    stop_event = threading.Event()
    spin_thread = threading.Thread(
        target=_spin_executor,
        args=(executor, stop_event),
        daemon=True,
    )
    spin_thread.start()

    root = tk.Tk()
    app = GripperControlApp(root, node, executor, spin_thread, stop_event)
    root.mainloop()


if __name__ == '__main__':
    main()
