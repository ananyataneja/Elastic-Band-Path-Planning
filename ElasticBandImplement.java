import javafx.animation.KeyFrame;
import javafx.animation.Timeline;
import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.canvas.*;
import javafx.scene.paint.*;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import javafx.util.Duration;
import javafx.scene.control.TextInputDialog;

import java.util.*;

public class ElasticBandImplement extends Application {

    static Bot bot;
    static List<Obs> obs = new ArrayList<>();

    class Vec {
        double x, y;
        Vec(double x, double y) { this.x = x; this.y = y; }
        double dis(Vec v) { return Math.hypot(x - v.x, y - v.y); }
        Vec copy() { return new Vec(x, y); }
    }

    class Obs {
        Vec pos;
        double rad, spd, dir;
        Obs(double x, double y, double r, double s, double d) {
            pos = new Vec(x, y);
            rad = r;
            spd = s;
            dir = d;
        }

        void upd(double dt) {
            double a = Math.toRadians(dir);
            pos.x += spd * Math.cos(a) * dt;
            pos.y += spd * Math.sin(a) * dt;
            if (pos.x < rad || pos.x > 12 - rad) dir = 180 - dir;
            if (pos.y < rad || pos.y > 12 - rad) dir = -dir;
        }

        Vec predict(double t) {
            double a = Math.toRadians(dir);
            return new Vec(pos.x + spd * Math.cos(a) * t, pos.y + spd * Math.sin(a) * t);
        }
    }

    class Bot {
        Vec pos, tgt;
        double rad;
        Bot(double x, double y, double r, double tx, double ty) {
            pos = new Vec(x, y);
            rad = r;
            tgt = new Vec(tx, ty);
        }
    }

    final int scl = 50;
    final double safe = 0.8, dt = 0.3;
    final double robotSpeed = 0.12;
    List<Vec> elasticBand = new ArrayList<>();

    @Override
    public void start(Stage stg) {
        // === Robot position and radius ===
        TextInputDialog dialog1 = new TextInputDialog("1 1 0.3");
        dialog1.setHeaderText("Enter robot position X Y and radius (space separated):");
        String[] parts1 = dialog1.showAndWait().orElse("1 1 0.3").split("\\s+");
        double bx = Double.parseDouble(parts1[0]);
        double by = Double.parseDouble(parts1[1]);
        double br = Double.parseDouble(parts1[2]);

        // === Goal position ===
        TextInputDialog dialog2 = new TextInputDialog("10 10");
        dialog2.setHeaderText("Enter goal position X Y:");
        String[] parts2 = dialog2.showAndWait().orElse("10 10").split("\\s+");
        double gx = Double.parseDouble(parts2[0]);
        double gy = Double.parseDouble(parts2[1]);

        bot = new Bot(bx, by, br, gx, gy);

        // === Number of obstacles ===
        TextInputDialog dialog3 = new TextInputDialog("0");
        dialog3.setHeaderText("Enter number of obstacles:");
        int n = Integer.parseInt(dialog3.showAndWait().orElse("0"));

        for (int i = 0; i < n; i++) {
            TextInputDialog oDialog = new TextInputDialog("5 5 0.5 0.0 0.0");
            oDialog.setHeaderText("Enter obstacle " + (i+1) + " position X Y, radius, speed, direction:");
            String[] oParts = oDialog.showAndWait().orElse("5 5 0.5 0.0 0.0").split("\\s+");

            double x = Double.parseDouble(oParts[0]);
            double y = Double.parseDouble(oParts[1]);
            double r = Double.parseDouble(oParts[2]);
            double s = Double.parseDouble(oParts[3]);
            double d = Double.parseDouble(oParts[4]);

            obs.add(new Obs(x, y, r, s, d));
        }

        // === Setup canvas and simulation ===
        Canvas can = new Canvas(600, 600);
        GraphicsContext gc = can.getGraphicsContext2D();
        Pane pane = new Pane(can);
        stg.setScene(new Scene(pane));
        stg.setTitle("Elastic Band Path Planner");
        stg.show();

        List<Vec> path = AStar(bot.pos, bot.tgt, 0.5);
        initElasticBand(path);

        Timeline tl = new Timeline(new KeyFrame(Duration.millis(50), e -> {
            for (Obs o : obs) o.upd(dt);
            deformElasticBand(obs);
            updateRobotPosition();
            draw(gc);
        }));
        tl.setCycleCount(Timeline.INDEFINITE);
        tl.play();
    }

    void initElasticBand(List<Vec> path) {
        elasticBand.clear();
        elasticBand.addAll(path);
    }

    void updateRobotPosition() {
        if (elasticBand.size() < 2) return;
        Vec next = elasticBand.get(1);
        double dist = bot.pos.dis(next);

        if (dist < robotSpeed) {
            bot.pos = next;
            elasticBand.remove(0);
        } else {
            double dx = next.x - bot.pos.x;
            double dy = next.y - bot.pos.y;
            double angle = Math.atan2(dy, dx);
            bot.pos.x += robotSpeed * Math.cos(angle);
            bot.pos.y += robotSpeed * Math.sin(angle);
        }

        if (!elasticBand.isEmpty()) {
            elasticBand.set(0, new Vec(bot.pos.x, bot.pos.y));
        }
    }

    void deformElasticBand(List<Obs> os) {
        double alpha = 0.2;
        double beta = 3.5;
        double gamma = 5.0;
        double minDist = safe + bot.rad;

        for (int i = 1; i < elasticBand.size() - 1; i++) {
            Vec prev = elasticBand.get(i - 1);
            Vec curr = elasticBand.get(i);
            Vec next = elasticBand.get(i + 1);

            double fx = 0, fy = 0;
            fx += alpha * (prev.x + next.x - 2 * curr.x);
            fy += alpha * (prev.y + next.y - 2 * curr.y);

            for (Obs o : os) {
                for (double t = 0; t <= gamma; t += 0.5) {
                    Vec fut = o.predict(t);
                    double dx = curr.x - fut.x;
                    double dy = curr.y - fut.y;
                    double dist = Math.hypot(dx, dy);
                    if (dist < minDist) dist = minDist;
                    double repulsion = beta / (dist * dist);
                    fx += repulsion * (dx / dist);
                    fy += repulsion * (dy / dist);
                }
            }

            curr.x += fx * 0.1;
            curr.y += fy * 0.1;
        }
    }

    void draw(GraphicsContext gc) {
        gc.clearRect(0, 0, 600, 600);

        gc.setFill(Color.RED);
        gc.fillOval(bot.tgt.x * scl - 5, bot.tgt.y * scl - 5, 10, 10);

        for (Obs o : obs) {
            RadialGradient gradient = new RadialGradient(0, 0, o.pos.x * scl, o.pos.y * scl,
                    o.rad * scl, false, CycleMethod.NO_CYCLE,
                    new Stop(0, Color.rgb(255, 165, 0, 0.8)),
                    new Stop(1, Color.rgb(255, 165, 0, 0.0)));
            gc.setFill(gradient);
            gc.fillOval(o.pos.x * scl - o.rad * scl, o.pos.y * scl - o.rad * scl,
                        o.rad * 2 * scl, o.rad * 2 * scl);
        }

        gc.setStroke(Color.BLUE);
        gc.setLineWidth(2);
        for (int i = 1; i < elasticBand.size(); i++) {
            Vec a = elasticBand.get(i - 1), b = elasticBand.get(i);
            gc.strokeLine(a.x * scl, a.y * scl, b.x * scl, b.y * scl);
        }

        gc.setFill(Color.GREEN);
        gc.fillOval(bot.pos.x * scl - bot.rad * scl, bot.pos.y * scl - bot.rad * scl,
                    bot.rad * 2 * scl, bot.rad * 2 * scl);
    }

    List<Vec> AStar(Vec start, Vec goal, double res) {
        int W = (int)(12 / res), H = (int)(12 / res);
        boolean[][] vis = new boolean[W][H];
        Vec[][] parent = new Vec[W][H];
        PriorityQueue<Node> pq = new PriorityQueue<>(Comparator.comparingDouble(a -> a.f));
        int sx = (int)(start.x / res), sy = (int)(start.y / res);
        int gx = (int)(goal.x / res), gy = (int)(goal.y / res);
        pq.add(new Node(sx, sy, 0, dist(sx, sy, gx, gy)));

        while (!pq.isEmpty()) {
            Node cur = pq.poll();
            if (vis[cur.x][cur.y]) continue;
            vis[cur.x][cur.y] = true;
            if (cur.x == gx && cur.y == gy) break;

            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    if (dx == 0 && dy == 0) continue;
                    int nx = cur.x + dx, ny = cur.y + dy;
                    if (nx < 0 || ny < 0 || nx >= W || ny >= H || vis[nx][ny]) continue;

                    double x = nx * res, y = ny * res;
                    boolean safe = true;
                    for (Obs o : obs) {
                        if (new Vec(x, y).dis(o.pos) < o.rad + bot.rad + 0.5) {
                            safe = false;
                            break;
                        }
                    }

                    if (!safe) continue;

                    parent[nx][ny] = new Vec(cur.x, cur.y);
                    double g = cur.g + dist(cur.x, cur.y, nx, ny);
                    double h = dist(nx, ny, gx, gy);
                    pq.add(new Node(nx, ny, g, g + h));
                }
            }
        }

        LinkedList<Vec> path = new LinkedList<>();
        int cx = gx, cy = gy;
        while (!(cx == sx && cy == sy)) {
            path.addFirst(new Vec(cx * res, cy * res));
            Vec p = parent[cx][cy];
            if (p == null) break;
            cx = (int)p.x;
            cy = (int)p.y;
        }
        path.addFirst(new Vec(sx * res, sy * res));
        return path;
    }

    double dist(int x1, int y1, int x2, int y2) {
        return Math.hypot(x1 - x2, y1 - y2);
    }

    class Node {
        int x, y;
        double g, f;
        Node(int x, int y, double g, double f) {
            this.x = x;
            this.y = y;
            this.g = g;
            this.f = f;
        }
    }

    public static void main(String[] args) {
        launch(args);
    }
}
