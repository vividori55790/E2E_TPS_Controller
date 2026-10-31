function verify_steady_eff_batch(pointsCsv, outCsv, chunk, simTime)
% verify_steady_eff_batch.m -- verify_steady_eff.m in chunks, same file layout and restart rule as verify_steady_fine_batch.m:
% <outCsv without .csv>_partNNN.csv per chunk, a chunk whose output exists is skipped.
    if nargin < 3 || isempty(chunk),   chunk = 6; end
    if nargin < 4 || isempty(simTime), simTime = 0.020; end
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
            fprintf('verify_steady_eff_batch: runs %d-%d of %d already in %s, skipped\n', s, e, N, pout);
            continue;
        end
        writetable(pts(s:e, :), pin);
        fprintf('verify_steady_eff_batch: runs %d-%d of %d -> %s (%s)\n', s, e, N, pout, char(datetime('now')));
        verify_steady_eff(pin, pout, simTime);
        delete(pin);
    end
    fprintf('verify_steady_eff_batch: done (%d runs)\n', N);
end
