/* Student Python runs off the UI thread. Terminating this worker stops even an
   infinite Python loop; no server executes submitted code. */
"use strict";
const RUNTIME = "https://cdn.jsdelivr.net/pyodide/v314.0.7/full/";

self.onmessage = async ({ data }) => {
  try {
    self.postMessage({ type: "status", message: "Downloading Python runtime…" });
    const { loadPyodide } = await import(RUNTIME + "pyodide.mjs");
    let outputLines = 0;
    const output = (text) => {
      if (outputLines++ < 150) self.postMessage({ type: "stdout", message: text.slice(0, 2000) });
    };
    const py = await loadPyodide({ indexURL: RUNTIME, stdout: output, stderr: output });
    self.postMessage({ type: "status", message: "Loading NumPy…" });
    await py.loadPackage("numpy");
    const runnerURL = new URL("runner.py", self.location.href);
    runnerURL.search = new URL(self.location.href).search;
    const response = await fetch(runnerURL, { cache: "no-cache" });
    if (!response.ok) throw new Error("Could not load the Python sequence runner.");
    await py.runPythonAsync(await response.text());
    py.globals.set("_student_code", data.code);
    py.globals.set("_clouds_json", JSON.stringify(data.clouds));
    py.globals.set("_report_frame", (payload) => {
      self.postMessage({ type: "frame", result: JSON.parse(payload) });
    });
    self.postMessage({ type: "running" });
    await py.runPythonAsync(`
import json
_student = {"__name__": "student", "np": np}
exec(compile(_student_code, "registration.py", "exec"), _student)
_register = _student.get("registration")
if not callable(_register):
    raise ValueError("Define a function named registration(source, target).")
_clouds = [np.asarray(points, dtype=float) for points in json.loads(_clouds_json)]
run_sequence(_clouds, _register, lambda frame: _report_frame(json.dumps(frame)))
`);
    self.postMessage({ type: "done" });
  } catch (error) {
    self.postMessage({ type: "error", message: String(error.message || error) });
  }
};
