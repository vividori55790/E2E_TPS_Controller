function verify_steady_batch(pointsCsv, outCsv, chunk, simTime)
% verify_steady_batch.m -- verify_steady_state.m in chunks of `chunk` runs (default 24).
% One parsim call collects every SimulationOutput in the client (~94 MB each for a 30 ms run); 92 at once ran out of
% memory during deserialisation on 2026-10-26 (sw1b: "parsim failed (... memory); running sequentially"). Chunks keep
% the peak near chunk x 94 MB; the parallel pool is reused between chunks. Paths are relative to the caller's folder.
% Output: one file per chunk, <outCsv without .csv>_partNNN.csv (simulink_sweep.py reads them with the same glob);
% they are not concatenated here (the error column's type can differ between chunks).
% Restart-safe (2026-10-29, after a reboot stopped r5r6 mid-chunk): a chunk whose output file already exists is skipped,
% so a resubmitted job with the same points file and chunk size continues where it stopped.
%
%   python automation/job.py submit NAME --interactive -- matlab -batch "cd('Simulink/Thesis'); verify_steady_batch('out/sweep_points_r2.csv', 'out/simulink_sweep_r2.csv')"

    if nargin < 3 || isempty(chunk),   chunk = 24; end
    if nargin < 4 || isempty(simTime), simTime = 0.030; end
    here = pwd;
    if ~(numel(pointsCsv) > 1 && pointsCsv(2) == ':'), pointsCsv = fullfile(here, pointsCsv); end
    if ~(numel(outCsv) > 1 && outCsv(2) == ':'),       outCsv = fullfile(here, outCsv); end
    pts = readtable(pointsCsv, 'TextType', 'string');
    N = height(pts);
    [d, b] = fileparts(outCsv);
    for s = 1:chunk:N
        e = min(N, s + chunk - 1);
        pin = fullfile(d, sprintf('%s_part%03d_in.csv', b, s));
        pout = fullfile(d, sprintf('%s_part%03d.csv', b, s));
        if isfile(pout)
            fprintf('verify_steady_batch: runs %d-%d of %d already in %s, skipped\n', s, e, N, pout);
            continue;
        end
        writetable(pts(s:e, :), pin);
        fprintf('verify_steady_batch: runs %d-%d of %d -> %s\n', s, e, N, pout);
        verify_steady_state(pin, pout, simTime);
        delete(pin);
    end
    fprintf('verify_steady_batch: done (%d runs)\n', N);
end
