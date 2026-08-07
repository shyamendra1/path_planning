import subprocess

class Run_process:
    def __init__(self, path, proc_name, password, args=None):
        self.path = path
        self.pwd = password
        self.process = None
        self.proc_name = proc_name
        self.args = args if args else []

    def runobject(self):
        try:
            cmd = [self.proc_name] + self.args

            print("Running:", cmd)

            self.process = subprocess.Popen(
                cmd,
                cwd=self.path,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            

            print(f"Started process '{self.proc_name}' with PID: {self.process.pid}")
            return self.proc_name

        except Exception as e:
            print(f"Failed to start process '{self.proc_name}':", e)
            return False

    def stopobject(self):
        try:
            self.process.terminate()
            self.process.wait(timeout=5) 
            #subprocess.run(["pkill", "-f", self.proc_name], check=True)
            print("stopped")
            return f"Stopped process '{self.proc_name}'."

        except subprocess.CalledProcessError:
            return f"No process named '{self.proc_name}' was running."

        except Exception as e:
            return f"Error stopping process '{self.proc_name}': {e}"
