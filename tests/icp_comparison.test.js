/* Run in the workshop browser console:
   eval(await (await fetch('tests/icp_comparison.test.js')).text()) */
(() => {
  const I = () => [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
  const close = (actual, expected, label) => {
    if (Math.abs(actual - expected) > 1e-6) throw new Error(`${label}: expected ${expected}, got ${actual}`);
  };
  const evaluate = ICPComparison.evaluate;
  const empty = evaluate([I()], []);
  if (empty.rmse !== null || empty.rows.length) throw new Error("Uncomputed poses must not count as zero error.");
  const identical = evaluate([I()], [{ index: 0, pose: I() }]);
  close(identical.rmse, 0, "Identical pose RMSE");
  close(identical.rows[0].rotationError, 0, "Identical rotation");
  const moved = I(); moved[0][3] = 3; moved[1][3] = 4; moved[2][3] = 12;
  const translation = evaluate([I(), I(), I()], [{ index: 0, pose: I() }, { index: 1, pose: moved }]);
  close(translation.rows[1].positionError, 13, "3D error includes Z");
  close(translation.rmse, 13 / Math.sqrt(2), "Partial RMSE includes origin but not uncomputed poses");
  const roll = [[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]];
  close(evaluate([I()], [{ index: 0, pose: roll }]).rows[0].rotationError, 90, "Full rotation includes roll");
  const halfTurn = [[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
  close(evaluate([I()], [{ index: 0, pose: halfTurn }]).rows[0].rotationError, 180, "180 degree rotation");
  close(evaluate([roll], [{ index: 0, pose: roll }]).rows[0].rotationError, 0, "Compare relative to nonidentity truth");
  const roundoff = I(); roundoff[0][0] += 1e-12;
  close(evaluate([I()], [{ index: 0, pose: roundoff }]).rows[0].rotationError, 0, "Clamp acos roundoff");
  return "PASS: ground-truth metrics, 3D errors, partial results, rotation and roundoff";
})();
