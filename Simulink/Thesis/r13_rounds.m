function r13_rounds(part, r0, r1, py)
% r13_rounds.m -- R13 stage 1 (r13_map.py): the 25 ns Simulink map in rounds without a loop turn in between, as
% fine_rounds.m does for GRID. part 'a' = set X + the map's SPS / M-B / M-S, 'b' = the stencil arms. Round 1 runs
% out/r13<part>_points_r1.csv (from `r13_map.py make --part <part>`; an error if it is missing); every later round
% first runs `r13_map.py next --part <part>`, which writes out/r13<part>_points_r<rnd>.csv with more D0 levels for the
% configurations whose measured powers do not bracket |P| yet, and stops at the first round with nothing to run.
% verify_steady_fine_batch: 12 runs per chunk, 20 ms (the GRID runner, window and per-run Ts column).
% Restart-safe: a chunk whose output exists is skipped, and `next` only lists what is still unbracketed.
%   python automation/job.py submit r13a --interactive --after r12r1b -- matlab -batch "cd('Simulink/Thesis'); r13_rounds('a', 1, 5, '<python.exe>')"
    if nargin < 1 || isempty(part), part = 'a'; end
    if nargin < 2 || isempty(r0), r0 = 1; end
    if nargin < 3 || isempty(r1), r1 = 5; end
    if nargin < 4 || isempty(py), py = 'python'; end
    part = char(part);
    here = fileparts(mfilename('fullpath'));
    out = fullfile(here, 'out');
    script = fullfile(here, 'r13_map.py');
    for rnd = r0:r1
        pts = fullfile(out, sprintf('r13%s_points_r%d.csv', part, rnd));
        if rnd > 1
            [st, msg] = system(sprintf('"%s" -I -X utf8 "%s" next --part %s', py, script, part));
            fprintf('%s', msg);
            if st ~= 0, error('r13_rounds: r13_map.py next failed (exit %d)', st); end
        elseif ~isfile(pts)
            % no automatic `make`: moving this file away is how a queued job is stopped (job.py kill cannot)
            error('r13_rounds: %s not found (run `r13_map.py make --part %s` first)', pts, part);
        end
        if ~isfile(pts)
            fprintf('r13_rounds: part %s round %d has nothing to run, done %s\n', part, rnd, datestr(now));
            return;
        end
        verify_steady_fine_batch(pts, fullfile(out, sprintf('r13%s_sweep_r%d.csv', part, rnd)), 12, 0.020);
    end
    fprintf('r13_rounds: part %s stopped after round %d %s\n', part, r1, datestr(now));
end
