package org.firstinspires.ftc.teamcode.lib;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.util.*;
import java.util.function.*;
import java.util.stream.*;

public class Quad<T> implements Collection<T>{
    public T w, x, y, z;

    public Quad(T w, T x, T y, T z){
        this.w = w;
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public static <T> Quad<T> of(T w, T x, T y, T z){
        return new Quad<>(w, x, y, z);
    }

    @Override
    public int size(){
        return 4;
    }

    @Override
    public boolean isEmpty(){
        return false;
    }

    @Override
    public boolean contains(@Nullable Object o){
        for(T item : this){
            if (item.equals(o)){
                return true;
            }
        }
        return false;
    }

    @NonNull
    @Override
    public Iterator<T> iterator() {
        return new QuadIterator();
    }

    @NonNull
    @Override
    public Object[] toArray(){
        return new Object[]{w, x, y, z};
    }

    @NonNull
    @Override
    public <E> E[] toArray(E[] a) {
        if (a.length < 4)
            return (E[]) toArray();
        if (a.length != 4) Arrays.fill(a, 4, a.length - 1, null);

        a[0] = (E) w;
        a[1] = (E) x;
        a[2] = (E) y;
        a[3] = (E) z;
        return a;
    }

    private final class QuadIterator implements Iterator<T> {
        private int index = 0;

        @Override
        public boolean hasNext() {
            return index < 4;
        }

        @Override
        public T next() {
            if (hasNext())
                switch (index++) {
                    case 0:
                        return w;
                    case 1:
                        return x;
                    case 2:
                        return y;
                    case 3:
                        return z;
                    default:
                        throw new NoSuchElementException();
                }
            else throw new NoSuchElementException();
        }
    }

    public static <T> Quad<T> repeat(T value){
        return of(value, value, value, value);
    }

    public static <P, Q, H> Quad<H> apply(
            Quad<P> quad1, Quad<Q> quad2, BiFunction<P, Q, H> function
    ){
        H w = function.apply(quad1.w, quad2.w);
        H x = function.apply(quad1.x, quad2.x);
        H y = function.apply(quad1.y, quad2.y);
        H z = function.apply(quad1.z, quad2.z);
        return new Quad<>(w, x, y, z);
    }
    public static <P, Q, H> Quad<H> apply(
            Quad<P> quad, Q q, BiFunction<P, Q, H> function
    ){
        H w = function.apply(quad.w, q);
        H x = function.apply(quad.x, q);
        H y = function.apply(quad.y, q);
        H z = function.apply(quad.z, q);
        return new Quad<>(w, x, y, z);
    }
    public <P, H> Quad<H> apply(
            Quad<P> quad, BiFunction<T, P, H> function
    ){
        return apply(this, quad, function);
    }
    public <P, H> Quad<H> apply(
            P p, BiFunction<T, P, H> function
    ){
        return apply(this, p, function);
    }
    public <R> Quad<R> apply(
            Function<T, R> function
    ){
        R w = function.apply(this.w);
        R x = function.apply(this.x);
        R y = function.apply(this.y);
        R z = function.apply(this.z);
        return of(w, x, y, z);
    }
    public static <P, Q> Quad<Q> applyAll(
            List<Quad<P>> quads, Function<List<P>, Q> function
    ){
        List<P> valuesW = quads.stream().map(quad -> quad.w).collect(Collectors.toList());
        List<P> valuesX = quads.stream().map(quad -> quad.x).collect(Collectors.toList());
        List<P> valuesY = quads.stream().map(quad -> quad.y).collect(Collectors.toList());
        List<P> valuesZ = quads.stream().map(quad -> quad.z).collect(Collectors.toList());

        Q w = function.apply(valuesW);
        Q x = function.apply(valuesX);
        Q y = function.apply(valuesY);
        Q z = function.apply(valuesZ);

        return of(w, x, y, z);
    }

    public static Quad<Double> scale(Quad<Double> quad, double scalar){
        return quad.apply(scalar, (a, b) -> a * b);
    }
    public Quad<Double> scaleBy(double scalar){
        return scale((Quad<Double>) this, scalar);
    }
    public static Quad<Double> sum(List<Quad<Double>> quads){
        return Quad.applyAll(quads, list -> list.stream().mapToDouble(Double::doubleValue).sum());
    }


    @Override
    public boolean containsAll(Collection<?> c) {
        return c.stream().allMatch(this::contains);
    }

    @Override
    public boolean add(T t) {
        throw new UnsupportedOperationException("Adding elements is not supported for Quad");
    }

    @Override
    public boolean remove(Object o) {
        throw new UnsupportedOperationException("Removing elements is not supported for Quad");
    }

    @Override
    public boolean addAll(Collection<? extends T> c) {
        throw new UnsupportedOperationException("Adding elements is not supported for Quad");
    }

    @Override
    public boolean removeAll(Collection<?> c) {
        throw new UnsupportedOperationException("Removing elements is not supported for Quad");
    }

    @Override
    public boolean retainAll(Collection<?> c) {
        throw new UnsupportedOperationException("Retaining elements is not supported for Quad");
    }

    @Override
    public void clear() {
        throw new UnsupportedOperationException("Clearing elements is not supported for Quad");
    }
}