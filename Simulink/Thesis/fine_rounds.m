function fine_rounds(r0, r1, py)
% fine_rounds.m -- GRID follow-up rounds without a loop turn in between (as opt_rounds.m does for OPT): for
% rnd = r0..r1, `fine_grid_check.py next` writes out/fine_points_r<rnd>.csv with one more D0 level for every
% configuration whose measured powers do not bracket |P| yet (fine1b, 10-30: at 25 ns the same D0 level carries
% 40-70 W more than at 100 ns, so several 25 ns configurations sit below their first level), then
% verify_steady_fine_batch runs it (12 runs per chunk, 20 ms). Stops at the first round with nothing to run.
% Restart-safe: a chunk whose output exists is skipped, and `next` only lists what is still unbracketed.
%   python automation/job.py submit fineR --interactive --after fine1b -- matlab -batch "cd('Simulink/Thesis'); fine_rounds(2, 4, '<python.exe>')"
    if nargin < 1 || isempty(r0), r0 = 2; end
    if nargin < 2 || isempty(r1), r1 = 4; end
    if nargin < 3 || isempty(py), py = 'python'; end
    here = fileparts(mfilename('fullpath'));
    out = fullfile(here, 'out');
    for rnd = r0:r1
        pts = fullfile(out, sprintf('fine_points_r%d.csv', rnd));
        [st, msg] = system(sprintf('"%s" -I -X utf8 "%s" next', py, fullfile(here, 'fine_grid_check.py')));
        fprintf('%s', msg);
        if st ~= 0, error('fine_rounds: fine_grid_check.py next failed (exit %d)', st); end
        if ~isfile(pts)
            fprintf('fine_rounds: round %d has nothing to run, done %s\n', rnd, datestr(now));
            return;
        end
        verify_steady_fine_batch(pts, fullfile(out, sprintf('fine_sweep_r%d.csv', rnd)), 12, 0.020);
    end
    fprintf('fine_rounds: stopped after round %d %s\n', r1, datestr(now));
end
