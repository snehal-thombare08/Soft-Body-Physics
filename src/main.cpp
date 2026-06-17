#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <string>
#include <algorithm>

const int WIDTH = 1200, HEIGHT = 800;
const float GRAVITY = 800.f;
const float DAMPING = 0.98f;
const float RESTITUTION = 0.55f;
const float SPRING_STIFFNESS = 1800.f;
const float SPRING_DAMPING = 12.f;
const int   SOLVER_ITERS = 8;

float randF(float lo, float hi){ return lo+(hi-lo)*(rand()/(float)RAND_MAX); }

struct Point {
    float x,y,px,py,vx,vy;
    float mass;
    bool pinned;
    Point(float x,float y,float m=1.f)
        :x(x),y(y),px(x),py(y),vx(0),vy(0),mass(m),pinned(false){}
};

struct Spring {
    int a,b;
    float restLen;
    float stiffness;
    bool structural; // true=edge, false=diagonal shear
    Spring(int a,int b,float l,float k,bool s)
        :a(a),b(b),restLen(l),stiffness(k),structural(s){}
};

struct SoftBody {
    std::vector<Point> pts;
    std::vector<Spring> springs;
    sf::Color color;
    int rows,cols;

    // grid: rows x cols points, CELL_SIZE spacing
    SoftBody(float cx,float cy,int rows,int cols,float cellSize, sf::Color col)
        :rows(rows),cols(cols),color(col)
    {
        float ox=cx-(cols-1)*cellSize*0.5f;
        float oy=cy-(rows-1)*cellSize*0.5f;
        for(int r=0;r<rows;r++)
            for(int c=0;c<cols;c++)
                pts.emplace_back(ox+c*cellSize, oy+r*cellSize);

        auto idx=[&](int r,int c){return r*cols+c;};
        float diag=cellSize*1.41421f;

        for(int r=0;r<rows;r++)
            for(int c=0;c<cols;c++){
                // structural
                if(c+1<cols) springs.emplace_back(idx(r,c),idx(r,c+1),cellSize,SPRING_STIFFNESS,true);
                if(r+1<rows) springs.emplace_back(idx(r,c),idx(r+1,c),cellSize,SPRING_STIFFNESS,true);
                // shear
                if(r+1<rows&&c+1<cols){
                    springs.emplace_back(idx(r,c),idx(r+1,c+1),diag,SPRING_STIFFNESS*0.7f,false);
                    springs.emplace_back(idx(r+1,c),idx(r,c+1),diag,SPRING_STIFFNESS*0.7f,false);
                }
                // bend (skip-one)
                if(c+2<cols) springs.emplace_back(idx(r,c),idx(r,c+2),cellSize*2,SPRING_STIFFNESS*0.3f,false);
                if(r+2<rows) springs.emplace_back(idx(r,c),idx(r+2,c),cellSize*2,SPRING_STIFFNESS*0.3f,false);
            }
    }

    void update(float dt){
        // integrate
        for(auto& p:pts){
            if(p.pinned) continue;
            p.vy+=GRAVITY*dt;
            p.vx*=DAMPING; p.vy*=DAMPING;
            p.x+=p.vx*dt; p.y+=p.vy*dt;
        }
        // spring solver
        for(int iter=0;iter<SOLVER_ITERS;iter++){
            for(auto& s:springs){
                Point& a=pts[s.a]; Point& b=pts[s.b];
                float dx=b.x-a.x, dy=b.y-a.y;
                float dist=std::sqrt(dx*dx+dy*dy);
                if(dist<0.0001f) continue;
                float err=(dist-s.restLen)/dist;
                float fx=dx*err*s.stiffness*dt*0.5f;
                float fy=dy*err*s.stiffness*dt*0.5f;
                float relVx=b.vx-a.vx, relVy=b.vy-a.vy;
                float damp=SPRING_DAMPING*(relVx*(dx/dist)+relVy*(dy/dist));
                float dfx=dx/dist*damp*dt*0.5f;
                float dfy=dy/dist*damp*dt*0.5f;
                if(!a.pinned){a.vx+=(fx+dfx)/a.mass; a.vy+=(fy+dfy)/a.mass;}
                if(!b.pinned){b.vx-=(fx+dfx)/b.mass; b.vy-=(fy+dfy)/b.mass;}
            }
        }
        // wall collisions
        for(auto& p:pts){
            if(p.pinned) continue;
            if(p.x<p.mass){            p.x=p.mass;    p.vx=-p.vx*RESTITUTION;}
            if(p.x>WIDTH-p.mass){      p.x=WIDTH-p.mass; p.vx=-p.vx*RESTITUTION;}
            if(p.y<p.mass){            p.y=p.mass;    p.vy=-p.vy*RESTITUTION;}
            if(p.y>HEIGHT-p.mass){     p.y=HEIGHT-p.mass; p.vy=-p.vy*RESTITUTION; p.vx*=0.85f;}
        }
    }

    // returns index of closest point, -1 if none within radius
    int hitTest(float mx,float my,float radius){
        int best=-1; float bestD=radius*radius;
        for(int i=0;i<(int)pts.size();i++){
            float dx=pts[i].x-mx, dy=pts[i].y-my;
            float d2=dx*dx+dy*dy;
            if(d2<bestD){bestD=d2;best=i;}
        }
        return best;
    }

    void applyImpulse(float mx,float my,float fx,float fy,float radius){
        for(auto& p:pts){
            float dx=p.x-mx,dy=p.y-my;
            float d2=dx*dx+dy*dy;
            if(d2<radius*radius){
                float t=1.f-std::sqrt(d2)/radius;
                p.vx+=fx*t/p.mass;
                p.vy+=fy*t/p.mass;
            }
        }
    }

    void draw(sf::RenderWindow& win, bool showSprings){
        if(showSprings){
            sf::VertexArray lines(sf::PrimitiveType::Lines);
            for(auto& s:springs){
                sf::Color sc=s.structural
                    ? sf::Color(color.r,color.g,color.b,160)
                    : sf::Color(color.r/2,color.g/2,color.b/2,60);
                lines.append(sf::Vertex{sf::Vector2f(pts[s.a].x,pts[s.a].y),sc});
                lines.append(sf::Vertex{sf::Vector2f(pts[s.b].x,pts[s.b].y),sc});
            }
            win.draw(lines);
        }

        // filled poly using triangle fan from centroid
        float cx=0,cy=0;
        for(auto& p:pts){cx+=p.x;cy+=p.y;}
        cx/=pts.size(); cy/=pts.size();

        // draw outline points as filled blob using triangles (outer ring only)
        // outer ring: top row, right col, bottom row reversed, left col reversed
        std::vector<int> outline;
        for(int c=0;c<cols;c++) outline.push_back(c);
        for(int r=1;r<rows;r++) outline.push_back(r*cols+(cols-1));
        for(int c=cols-2;c>=0;c--) outline.push_back((rows-1)*cols+c);
        for(int r=rows-2;r>=1;r--) outline.push_back(r*cols);

        sf::VertexArray tris(sf::PrimitiveType::Triangles);
        sf::Color fillCol(color.r,color.g,color.b,210);
        sf::Color centerCol(
            std::min(255,color.r+60),
            std::min(255,color.g+60),
            std::min(255,color.b+60),200);
        for(int i=0;i<(int)outline.size();i++){
            int j=(i+1)%outline.size();
            tris.append(sf::Vertex{sf::Vector2f(cx,cy),centerCol});
            tris.append(sf::Vertex{sf::Vector2f(pts[outline[i]].x,pts[outline[i]].y),fillCol});
            tris.append(sf::Vertex{sf::Vector2f(pts[outline[j]].x,pts[outline[j]].y),fillCol});
        }
        win.draw(tris);

        // draw point nodes
        sf::CircleShape dot(3.f,8);
        dot.setOrigin({3.f,3.f});
        for(auto& p:pts){
            dot.setPosition({p.x,p.y});
            dot.setFillColor(p.pinned ? sf::Color(255,200,50) : sf::Color(255,255,255,120));
            win.draw(dot);
        }
    }
};

std::vector<SoftBody> bodies;

void spawnBody(float cx,float cy){
    static sf::Color palette[]={
        sf::Color(80,180,255),
        sf::Color(255,100,120),
        sf::Color(80,220,140),
        sf::Color(220,160,255),
        sf::Color(255,200,60)
    };
    static int pi=0;
    int r=3+rand()%3, c=3+rand()%3;
    float cell=18.f+randF(-4,4);
    bodies.emplace_back(cx,cy,r,c,cell,palette[pi++%5]);
}

int main(){
    sf::RenderWindow window(sf::VideoMode({(unsigned)WIDTH,(unsigned)HEIGHT}),
        "Soft Body Physics | LMB: Spawn | RMB: Grab/Throw | Space: Explode | S: Springs | C: Clear | P: Pin");
    window.setFramerateLimit(60);

    srand(42);
    // spawn a few to start
    spawnBody(300,200); spawnBody(600,150); spawnBody(900,200);
    spawnBody(450,350); spawnBody(750,300);

    bool showSprings=true;
    bool dragging=false;
    int dragBody=-1, dragPoint=-1;
    sf::Vector2f prevMouse;

    sf::Font font;
    bool hasFont=font.openFromFile("C:/Windows/Fonts/arial.ttf");
    sf::Text hud(font);
    hud.setCharacterSize(14);
    hud.setFillColor(sf::Color::White);
    hud.setOutlineColor(sf::Color::Black);
    hud.setOutlineThickness(1.5f);
    hud.setPosition({10.f,10.f});

    sf::Clock clock;

    while(window.isOpen()){
        float dt=std::min(clock.restart().asSeconds(),0.02f);
        auto mpos=sf::Mouse::getPosition(window);
        sf::Vector2f mouse((float)mpos.x,(float)mpos.y);

        while(auto ev=window.pollEvent()){
            if(ev->is<sf::Event::Closed>()) window.close();
            if(auto* k=ev->getIf<sf::Event::KeyPressed>()){
                if(k->code==sf::Keyboard::Key::Escape) window.close();
                if(k->code==sf::Keyboard::Key::S) showSprings=!showSprings;
                if(k->code==sf::Keyboard::Key::C){ bodies.clear(); }
                if(k->code==sf::Keyboard::Key::Space){
                    // explosion at mouse
                    for(auto& b:bodies)
                        b.applyImpulse(mouse.x,mouse.y,0,-2200.f,180.f);
                }
                if(k->code==sf::Keyboard::Key::P){
                    // pin/unpin closest point of any body
                    float best=30.f*30.f; int bi=-1,pi2=-1;
                    for(int i=0;i<(int)bodies.size();i++){
                        int h=bodies[i].hitTest(mouse.x,mouse.y,30.f);
                        if(h>=0){
                            float dx=bodies[i].pts[h].x-mouse.x;
                            float dy=bodies[i].pts[h].y-mouse.y;
                            float d2=dx*dx+dy*dy;
                            if(d2<best){best=d2;bi=i;pi2=h;}
                        }
                    }
                    if(bi>=0) bodies[bi].pts[pi2].pinned=!bodies[bi].pts[pi2].pinned;
                }
            }
            if(auto* mb=ev->getIf<sf::Event::MouseButtonPressed>()){
                if(mb->button==sf::Mouse::Button::Left){
                    spawnBody(mouse.x,mouse.y);
                }
                if(mb->button==sf::Mouse::Button::Right){
                    for(int i=(int)bodies.size()-1;i>=0;i--){
                        int h=bodies[i].hitTest(mouse.x,mouse.y,40.f);
                        if(h>=0){dragging=true;dragBody=i;dragPoint=h;prevMouse=mouse;break;}
                    }
                }
            }
            if(auto* mb=ev->getIf<sf::Event::MouseButtonReleased>()){
                if(mb->button==sf::Mouse::Button::Right&&dragging&&dragBody>=0){
                    // throw
                    sf::Vector2f vel=(mouse-prevMouse)*60.f;
                    bodies[dragBody].applyImpulse(
                        bodies[dragBody].pts[dragPoint].x,
                        bodies[dragBody].pts[dragPoint].y,
                        vel.x,vel.y,60.f);
                    dragging=false;dragBody=-1;dragPoint=-1;
                }
            }
        }

        if(dragging&&dragBody>=0&&dragBody<(int)bodies.size()){
            auto& p=bodies[dragBody].pts[dragPoint];
            p.vx=(mouse.x-p.x)/dt*0.3f;
            p.vy=(mouse.y-p.y)/dt*0.3f;
            prevMouse=mouse;
        }

        for(auto& b:bodies) b.update(dt);

        window.clear(sf::Color(12,14,22));

        for(auto& b:bodies) b.draw(window,showSprings);

        // floor line
        sf::RectangleShape floor({(float)WIDTH,2.f});
        floor.setPosition({0.f,(float)HEIGHT-2.f});
        floor.setFillColor(sf::Color(60,65,80));
        window.draw(floor);

        if(hasFont){
            hud.setString(
                "LMB: Spawn Blob | RMB Drag: Grab/Throw | Space: Explode Up | S: Toggle Springs | P: Pin Point | C: Clear\n"
                "Bodies: "+std::to_string(bodies.size())
            );
            window.draw(hud);
        }
        window.display();
    }
    return 0;
}
