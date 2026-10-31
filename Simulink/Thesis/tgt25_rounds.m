function tgt25_rounds(part, r0, r1, py)
% tgt25_rounds.m -- the targets and structures on the 25 ns map (tgt25.py) in rounds without a loop turn in between, as
% r13_rounds.m does for the map. part 'a' = controller A, ICROS, B0; 'b' = LUT 49 x 22, D = 64, LUT 25 x 11, LUT 13 x 6. Round 1
% runs out/tgt25<part>_points_r1.csv (from `tgt25.py make`; an error if it is missing, which is how a queued job is stopped);
% every later round first runs `tgt25.py next --part <part>`, which writes out/tgt25<part>_points_r<rnd>.csv with more D0 levels
% for the configurations whose powers do not bracket |P| yet, and stops at the first round with nothing to run.
% verify_steady_fine_batch: 12 runs per chunk, 20 ms. Restart-safe: a chunk whose output exists is skipped.
%   python automation/job.py submit tgt25a --interactive --after v1e -- matlab -softwareopengl -batch "cd('Simulink/Thesis'); tgt25_rounds('a', 1, 5, '<python.exe>')"
    if nargin < 1 || isempty(part), part = 'a'; end
    if nargin < 2 || isempty(r0), r0 = 1; end
    if nargin < 3 || isempty(r1), r1 = 5; end
    if nargin < 4 || isempty(py), py = 'python'; end
    part = char(part);
    here = fileparts(mfilename('fullpath'));
    out = fullfile(here, 'out');
    script = fullfile(here, 'tgt25.py');
    for rnd = r0:r1
        pts = fullfile(out, sprintf('tgt25%s_points_r%d.csv', part, rnd));
        if rnd > 1
            [st, msg] = system(sprintf('"%s" -I -X utf8 "%s" next --part %s', py, script, part));
            fprintf('%s', msg);
            if st ~= 0, error('tgt25_rounds: tgt25.py next failed (exit %d)', st); end
        elseif ~isfile(pts)
            error('tgt25_rounds: %s not found (run `tgt25.py make` first)', pts);
        end
        if ~isfile(pts)
            fprintf('tgt25_rounds: part %s round %d has nothing to run, done %s\n', part, rnd, datestr(now));
            return;
        end
        verify_steady_fine_batch(pts, fullfile(out, sprintf('tgt25%s_sweep_r%d.csv', part, rnd)), 12, 0.020);
    end
    fprintf('tgt25_rounds: part %s stopped after round %d %s\n', part, r1, datestr(now));
end
